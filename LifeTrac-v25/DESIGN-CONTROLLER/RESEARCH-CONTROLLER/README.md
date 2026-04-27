# RESEARCH-CONTROLLER — archived prototypes and research

> **Treat everything in this folder as ideas/prior-art only \u2014 not implementation plans.** The canonical v25 build is LoRa-only, defined in [`../MASTER_PLAN.md`](../MASTER_PLAN.md). Code reviews and readiness analyses for the v25 hardware test do **not** apply to files under `RESEARCH-CONTROLLER/`.

This folder contains **superseded** controller designs and research notes that informed the current primary design (Portenta Max Carrier + MKR WAN 1310, see [../MASTER_PLAN.md](../MASTER_PLAN.md) and [../ARCHITECTURE.md](../ARCHITECTURE.md)).

Nothing here is the active design. Files are kept because they contain reusable hardware patterns, control logic, and research that the new firmware will draw on.

Recent additions to this folder (2026-04-26 cleanup):

- [`WIRELESS_OPTIONS.md`](WIRELESS_OPTIONS.md) \u2014 historical comparison of wireless technologies (XBee, ELRS, WiFi, cellular, LoRa, Meshtastic, etc.). LoRa was selected; this is fallback/reference material only.
- [`config/`](config/) \u2014 Mosquitto broker config and WiFi setup notes from the legacy MQTT-over-WiFi path.
- [`test_scripts/`](test_scripts/) \u2014 legacy MQTT bench-test scripts.
- [`AUTOMATION_AND_ROUTE_PLANNING.md`](AUTOMATION_AND_ROUTE_PLANNING.md) \u2014 future-release scope doc for autonomy, waypoint follow, coverage planning, and the wire-protocol slots reserved (but not implemented) in v25 to keep the door open.
- [`LATENCY_BUDGET.md`](LATENCY_BUDGET.md) \u2014 end-to-end latency analysis (operator stick \u2192 hydraulic actuation) and ranked optimization candidates. Several recommendations from this doc are already pinned in MASTER_PLAN \u00a78.17 / \u00a78.18.

---

## What's here, why it was archived, and what's still useful

### `arduino_opta_controller/`

**What it was:** Arduino Opta PLC running the tractor-side valve control, with a Modbus-style I/O expansion and Ethernet uplink.

**Why archived:** The Opta has no LoRa, no cellular, and limited expandability for our remote-control use case. The Portenta Max Carrier offers all of that on one board for similar cost.

**Still useful — harvest from this:**
- Valve drive sequencing and deadband logic — port directly to `firmware/tractor_h7/tractor_m4.cpp`
- Failsafe state-machine
- Modbus RTU integration patterns for valve drivers

### `esp32_remote_control/`

**What it was:** ESP32-based handheld using BLE to talk to a phone or directly to an Opta+BLE module.

**Why archived:** BLE range (~30 m line-of-sight, less in obstructed environments) is insufficient for a tractor remote. LoRa on the MKR WAN 1310 gives 1–2 km handheld range with lower power.

**Still useful — harvest from this:**
- Joystick scanning + debounce code → port to `firmware/handheld_mkr/inputs.cpp`
- OLED status-screen layout
- LiPo charging circuit reference

### `raspberry_pi_web_controller/`

**What it was:** Raspberry Pi 4 running nginx + Mosquitto + a Flask web UI as the base station, talking to the Opta over Ethernet/MQTT.

**Why archived:** The Portenta X8 runs the same stack (Yocto Linux + Docker), is industrial-rated, integrates with the Max Carrier for LoRa+cellular on one carrier, and has a deterministic M7+M4 co-processor. The Pi 4 + Opta + LoRa shield = three boards; the Max Carrier + X8 = one.

**Still useful — harvest from this:**
- Web UI HTML/CSS/JS (joystick widget, telemetry sidebar, map page) → port to `base_station/web_ui/`
- Mosquitto config → reuse via [../config/mosquitto.conf](../config/mosquitto.conf)
- nginx reverse-proxy config
- WebSocket message format design

### `ros2_bridge/`

**What it was:** Bridge from MQTT control topics to ROS 2 for robotics integration.

**Why archived:** Not abandoned — postponed to Phase 10 (stretch goals in [../TODO.md](../TODO.md)). The MQTT topic conventions that the new design uses are the same ones this bridge expects, so it can be revived without changes.

### `LORA_CUSTOM_STACK_TODO.md`

**What it was:** Earlier development plan for a custom LoRa stack on SparkFun LoRa Thing Plus + dedicated LoRa shield for the Opta.

**Why archived:** Subsumed by the Portenta Max Carrier (LoRa is built in) and superseded by [../TODO.md](../TODO.md). The protocol design (KISS framing, AES-128-GCM, MQTT-SN, RadioLib) was kept verbatim and is now in [../LORA_PROTOCOL.md](../LORA_PROTOCOL.md).

**Still useful — harvest from this:**
- Phased plan structure
- Acceptance-criteria tables
- FCC compliance notes

### `DROIDPAD_INTEGRATION.md` + `DROIDPAD_BLE_SETUP.md`

**What it was:** Integration notes for using the DroidPad Android app as a BLE controller for the Opta.

**Why archived:** Reduced to a stretch goal. With the new architecture, DroidPad would talk to the base station's web UI over WiFi (via WebSocket), making the BLE pairing dance unnecessary.

**Still useful — harvest from this:**
- DroidPad button mapping conventions
- Mobile-app UX patterns

### Wiring + change-log docs from the Opta era

`WIRING_DIAGRAM.md`, `MODE_SWITCH_WIRING.md`, `QUICK_REFERENCE_MODE_SWITCH.md`, `CHANGELOG_v25_MODE_SWITCH.md`, `MICROTRAC_V17.10_OPTIMIZATION.md`, `IMPLEMENTATION_SUMMARY.md`, `INSTALLATION_GUIDE.md`, `CODE_REVIEW.md`

**What they were:** Documentation supporting the Opta-based design.

**Why archived:** The new design has its own documentation tree under `../` (TRACTOR_NODE.md, BASE_STATION.md, HANDHELD_REMOTE.md). Any wiring conventions worth keeping (e.g., valve-driver pinouts) have been carried into the new docs.

**Still useful — harvest from this:**
- Mode-switch hardware design (a physical mode switch on the tractor's electrical box may be added back as a fourth control source in a future revision)
- Microtrac v17.10 optimization notes — relevant if a Microtrac-class small machine reuses any v25 controller hardware

---

## Resurrection guidance

If the primary Portenta-based design hits a blocker (e.g., supply-chain failure, undiscovered LoRa SiP issue, regulatory snag), here is the order to fall back through these alternatives:

1. **Replace MKR WAN 1310 handheld** with a SparkFun LoRa Thing Plus + custom enclosure — same Murata SiP, same firmware, slightly more wiring work
2. **Replace Portenta X8 base station** with a Raspberry Pi 4 + USB-to-LoRa dongle (Adafruit RFM95W FeatherWing on a USB carrier) — Docker compose stack runs unchanged
3. **Replace Portenta Max Carrier on the tractor** with an Arduino Opta + Adafruit RFM95W LoRa breakout + Particle Boron LTE — costs more in board count, but every component is independently sourceable

Maintaining this fallback path is why none of these files are deleted.

---

## See also

- [../README.md](../README.md) — primary design index
- [../ARCHITECTURE.md](../ARCHITECTURE.md) — what replaced what's here, and why
- [../TODO.md § Risk register](../TODO.md#risk-register) — formal risk list including supply-chain mitigation
