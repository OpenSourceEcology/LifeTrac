# Handheld Remote — Arduino MKR WAN 1310

The proximity-control handheld. Used by an operator working alongside the tractor. Holds the highest control priority of the three sources.

## Why MKR WAN 1310?

The MKR WAN 1310 carries the **same Murata CMWX1ZZABZ-078 LoRa SiP** as the Portenta Max Carrier. This means:

- Identical LoRa air interface — RadioLib code is portable verbatim from tractor to handheld
- Same regulatory profile (US 915 MHz / EU 868 MHz variants)
- Same power class (+14 dBm via the MKR's matching network; the Murata SiP itself supports up to +20 dBm but the MKR's onboard RF path is tuned for +14 dBm and a small whip antenna)

Plus, the MKR has:
- SAMD21 Cortex-M0+ at 48 MHz — plenty for joystick scanning + LoRa framing + OLED
- Onboard LiPo charger (just plug in a battery)
- USB-C native (program + serial in one cable)
- 22 GPIO + 7 ADC + 2 UART + I²C + SPI
- Small enough (67 × 25 mm) to fit a one-handed enclosure
- Cost: ~$45

## Hardware overview

| Component | Role |
|---|---|
| Arduino MKR WAN 1310 (ABX00029) | Main board with LoRa |
| 1/4-wave 915 MHz whip (u.FL or external SMA pigtail) | Antenna |
| Dual-axis analog joystick (×2) | Drive (left), implement/boom (right) |
| 22 mm latching mushroom E-stop | Hardware-direct E-stop signal |
| Momentary push buttons (×4) | TAKE CONTROL, mode toggle, AUX 1, AUX 2 |
| OLED display 128×64 SSD1306 (I²C) | Status: source priority, RSSI, battery |
| 1S 2000 mAh LiPo | ~8 h continuous operation |
| USB-C panel-mount connector | Charge + program |
| IP54 handheld enclosure | Hammond / Polycase, ~120 × 80 × 40 mm |

Full BOM in [HARDWARE_BOM.md § Tier 3](HARDWARE_BOM.md#tier-3--handheld-remote-mkr-wan-1310).

## Physical layout

```
                ┌────────────────────────────────┐
                │       128×64 OLED display       │
                │  ┌──────────────────────────┐  │
                │  │ HANDHELD: ACTIVE         │  │
                │  │ RSSI: -65 dBm   Loss: 0% │  │
                │  │ BATT: 78%       T+30s    │  │
                │  └──────────────────────────┘  │
                ├────────────────────────────────┤
                │                                 │
                │   ┌─────┐         ┌─────┐      │
                │   │ LH  │         │ RH  │      │
                │   │stick│         │stick│      │
                │   └─────┘         └─────┘      │
                │                                 │
                │  [TAKE   ] [MODE]  [AUX1]      │
                │  [CONTROL]         [AUX2]      │
                │                                 │
                │           ┌────────┐            │
                │           │  [ ]   │  E-STOP   │
                │           │  RED   │  (latching)│
                │           └────────┘            │
                │                                 │
                │  USB-C ◯       Power: ◯ on/off │
                └────────────────────────────────┘
```

## Wiring (MKR WAN 1310 pin assignments)

| MKR pin | Function | Connects to |
|---|---|---|
| A0 | LH joystick X (analog) | Left joystick wiper X |
| A1 | LH joystick Y (analog) | Left joystick wiper Y |
| A2 | RH joystick X (analog) | Right joystick wiper X |
| A3 | RH joystick Y (analog) | Right joystick wiper Y |
| A4 | LiPo voltage divider | 1/2 voltage divider on VBAT (built-in via `ADC_BATTERY` pin) |
| D0 | LH joystick click | Left joystick switch (pull-up) |
| D1 | RH joystick click | Right joystick switch (pull-up) |
| D2 | TAKE CONTROL momentary | Pull-up; press grounds |
| D3 | MODE momentary | Pull-up |
| D4 | AUX 1 momentary | Pull-up |
| D5 | AUX 2 momentary | Pull-up |
| D6 | E-STOP signal in | Latching mushroom; opens on press → digital read goes HIGH (pull-down) |
| D7 | (spare) | |
| D8 (MOSI) | (used internally for LoRa SPI) | |
| D9 (SCK) | (used internally for LoRa SPI) | |
| D10 (MISO) | (used internally for LoRa SPI) | |
| SDA / SCL | OLED I²C | SSD1306 SDA/SCL |
| 3V3 | OLED + joystick supply | 3.3 V rail |
| GND | common ground | |
| VBAT | LiPo direct connection | 1S LiPo + terminal |
| GND | LiPo − | |

## Firmware structure

### Main sketch (`handheld.ino`)

```c
#include <MKRWAN.h>      // Murata LoRa via Arduino library; OR use RadioLib for full control
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "lora_proto.h"
#include "crypto.h"

SX1276 lora = new Module(LORA_SS, LORA_DIO0, LORA_RST, LORA_DIO1);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

ControlFrame cf;
HeartbeatFrame hb;
uint8_t take_control_held = 0;
uint32_t take_control_release_ms = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    
    int state = lora.begin(915.0, 500.0, 7, 5, 0x12, 14, 8);
    if (state != RADIOLIB_ERR_NONE) {
        // fail — show error on OLED
    }
    
    crypto_init(PRESHARED_KEY);
    setup_pins();
}

void loop() {
    static uint32_t last_control_ms = 0;
    static uint32_t last_hb_ms = 0;
    static uint32_t last_display_ms = 0;
    uint32_t now = millis();
    
    // 1. Read inputs at 50 Hz
    if (now - last_control_ms >= 20) {
        read_joysticks(&cf);
        read_buttons(&cf);
        check_estop(&cf);
        update_take_control(&cf, now);
        
        cf.source_id = SOURCE_HANDHELD;
        cf.frame_type = FRAME_CONTROL;
        cf.sequence_num++;
        
        uint8_t buf[64];
        size_t len = lora_proto_encode(&cf, buf);
        lora.startTransmit(buf, len);
        last_control_ms = now;
    }
    
    // 2. Heartbeat at 20 Hz (interleaved with control frames; control-only is enough,
    //    but explicit heartbeat lets us claim priority even if joysticks aren't moving)
    if (now - last_hb_ms >= 50) {
        // Most ControlFrames serve as heartbeats; only send dedicated HB when idle
        if (joysticks_idle() && now - last_control_ms > 200) {
            send_heartbeat();
        }
        last_hb_ms = now;
    }
    
    // 3. Display refresh at 5 Hz
    if (now - last_display_ms >= 200) {
        update_display();
        last_display_ms = now;
    }
    
    // 4. Listen for telemetry / acks (not required for control, but useful for RSSI display)
    if (lora.available()) {
        uint8_t buf[64];
        size_t len = lora.readData(buf, sizeof(buf));
        Frame rx;
        if (lora_proto_decode(buf, len, &rx) == OK) {
            // Update RSSI / link state for display
        }
    }
}

void update_take_control(ControlFrame* cf, uint32_t now) {
    bool button_pressed = digitalRead(PIN_TAKE_CONTROL) == LOW;
    
    if (button_pressed) {
        take_control_held = 1;
        take_control_release_ms = 0;
    } else if (take_control_held) {
        if (take_control_release_ms == 0) {
            take_control_release_ms = now;
        }
        if (now - take_control_release_ms < 30000) {
            // 30 s latch
            take_control_held = 1;
        } else {
            take_control_held = 0;
        }
    }
    
    cf->flags = 0;
    if (take_control_held) cf->flags |= FLAG_TAKE_CONTROL;
    if (digitalRead(PIN_ESTOP) == HIGH) cf->flags |= FLAG_ESTOP_ARMED;
}
```

## E-stop wiring

The E-stop must signal even if the SAMD21 firmware is hung. Two paths:

1. **Software path** (normal): D6 pin reads E-stop button state, firmware sets `flags.estop_armed=0` in next ControlFrame, tractor receives and triggers failsafe.
2. **Hardware path** (failsafe-on-failsafe): the absence of valid heartbeats for 500 ms also triggers tractor failsafe. So pulling power on the handheld (e.g., dropping it) also triggers safe-state.

The handheld's E-stop button does NOT need to physically interrupt power to the MKR — that would prevent the firmware from sending the explicit E-stop frame. Instead, pressing the button is read as an input; firmware sends an E-stop ControlFrame at 50 Hz until released.

For *engine kill* (separate from valve neutralization), the tractor's H7 receives the E-stop frame and energizes the engine-kill solenoid via the safety relay.

## Battery & charging

- 1S LiPo, 2000 mAh = ~7.4 Wh
- Total handheld draw: ~80 mA continuous (LoRa TX 50% duty, OLED on) → **~25 hours** runtime
- Charge via USB-C, MKR's onboard MCP73831 charger (~500 mA charge rate → ~4 h full charge)
- Low-battery indication: OLED shows red "BATT LOW" below 20%; firmware reduces TX rate below 10%

## Bring-up checklist

1. [ ] MKR WAN 1310 + breadboard + serial monitor "hello world"
2. [ ] LoRa TX/RX with a second MKR or Max Carrier at 1 m bench distance
3. [ ] Wire joysticks, verify analog reads sweep -127 to +127 with deadband
4. [ ] Wire buttons + E-stop, verify all reads
5. [ ] OLED works, display refresh at 5 Hz
6. [ ] LiPo + onboard charger works; verify charge curve
7. [ ] Custom PCB or perfboard wiring assembly
8. [ ] Enclosure assembly with cable strain relief
9. [ ] Range test outdoors at 100 m, 500 m, 1 km, 2 km — record RSSI and packet loss
10. [ ] TAKE CONTROL latch test: hold button, release, verify 30 s latch on tractor side
11. [ ] E-stop test: press, verify tractor goes to failsafe within 200 ms
12. [ ] Drop test: power-off the handheld, verify tractor goes to failsafe within 500 ms
13. [ ] 8-hour battery life test in normal use pattern

## Source code layout (when implemented)

```
firmware/handheld_mkr/
├── handheld.ino                # Main sketch
├── inputs.cpp                  # Joystick reads, debounce, deadband
├── display.cpp                 # OLED status screen
├── pinmap_mkr_wan_1310.h       # Pin definitions
├── ../common/lora_proto.h      # Shared frame format
├── ../common/lora_proto.cpp    # Shared encoder/decoder
└── ../common/crypto.cpp        # Shared AES-GCM
```

## See also

- [HARDWARE_BOM.md § Tier 3](HARDWARE_BOM.md#tier-3--handheld-remote-mkr-wan-1310)
- [LORA_PROTOCOL.md § Take-control sequence](LORA_PROTOCOL.md#take-control-sequence-handheld)
- [TRACTOR_NODE.md](TRACTOR_NODE.md) — what the handheld talks to
- [Arduino MKR WAN 1310 docs](https://docs.arduino.cc/hardware/mkr-wan-1310)
- [RESEARCH-CONTROLLER/esp32_remote_control/](RESEARCH-CONTROLLER/esp32_remote_control/) — superseded ESP32 BLE remote (joystick scanning patterns reusable)
