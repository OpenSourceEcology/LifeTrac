# Capability inventory — LifeTrac v25 build configuration

> **Round 27 / BC-01.** Source-of-truth list of every LifeTrac v25
> capability that is **optional** (may be absent on a particular build)
> or **parameterised** (present on every build, but the spec varies).
> The companion JSON Schema at
> [`base_station/config/build_config.schema.json`](base_station/config/build_config.schema.json)
> encodes this table machine-readably; the Python loader at
> [`base_station/build_config.py`](base_station/build_config.py)
> consumes it.
>
> This document is consumed by humans designing variants and by the
> SIL test
> [`base_station/tests/test_build_config_loader_sil.py`](base_station/tests/test_build_config_loader_sil.py),
> which pins that every `optional`/`parameter` row below has a matching
> entry in the schema (and vice-versa, no orphans).

## How a capability ends up here

A line item in [`HARDWARE_BOM.md`](HARDWARE_BOM.md) qualifies if **any** of:

* Builders are documented as legitimately omitting it (e.g. the second /
  third / fourth camera, the optional implement-cam, the optional Coral
  TPU, the optional crop-health camera).
* The part has multiple acceptable substitutes with different spec
  (e.g. valve flow rating, IMU model, GPS receiver model).
* The wiring topology can vary (e.g. E-stop loop: monitored PSR vs
  hard-wired only vs PSR + redundant interlock).
* The firmware / base-station code today contains a hard-coded constant
  whose realistic field range is wider than one (axis count, camera
  count, watchdog thresholds).

A line item that is *required for safe operation* (PSR safety relay,
mushroom E-stop, M4 watchdog) is **not** optional but its parameters
(latency thresholds, fail-count latch) may still be tunable.

## Capability table (v1 — Round 27)

The `id` column is the JSON Schema property name. Defaults reflect the
canonical-BOM build documented in
[`HARDWARE_BOM.md`](HARDWARE_BOM.md). "Consumers" lists the modules
that read the capability.

### Identity

| id | type | default | range / enum | consumers | notes |
|---|---|---|---|---|---|
| `unit_id` | string | `lifetrac-001` | `^[a-z0-9-]{3,32}$` | `build_config.py`, audit log, MQTT client id | Per-fleet identifier; used to pick `build.<unit_id>.toml`. |
| `schema_version` | int | `1` | `>= 1` | `build_config.py` | Bumped when the schema breaks backward-compat. |

### Hydraulic axes

| id | type | default | range / enum | consumers | notes |
|---|---|---|---|---|---|
| `hydraulic.track_axis_count` | int | `2` | `1..2` | `tractor_h7.ino` (`apply_control`), `web_ui.py` joystick mapping | 2 = independent left/right tracks (canonical). 1 = single-axis throttle (rare; some builders without skid-steer). |
| `hydraulic.arm_axis_count` | int | `2` | `0..2` | `tractor_h7.ino`, `web_ui.py` arm UI | 2 = lift + tilt (canonical loader). 1 = lift only. 0 = no arm (drive-only chassis). |
| `hydraulic.proportional_flow` | bool | `true` | true/false | Opta `tractor_opta.ino`, `MASTER_TEST_PROGRAM.md` W4-05 | `true` requires the Opta A0602 + Burkert 8605 flow valve. `false` falls back to bang-bang directional valves (no ramp-out). |
| `hydraulic.track_ramp_seconds` | float | `2.0` | `0.5..5.0` | M7 `step_axis_ramp` | Per-axis tunable; W4-05 SIL pins the formula but the constant is per-build. |
| `hydraulic.arm_ramp_seconds` | float | `1.0` | `0.25..3.0` | M7 `step_axis_ramp` | Same. |

### E-stop topology

| id | type | default | range / enum | consumers | notes |
|---|---|---|---|---|---|
| `safety.estop_topology` | enum | `psr_monitored_dual` | `psr_monitored_dual` \| `psr_monitored_single` \| `hardwired_only` | M4 watchdog firmware, `tractor_opta.ino`, `MASTER_TEST_PROGRAM.md` W4-01/W4-03 | `psr_monitored_dual` = canonical Phoenix PSR-MC38 dual-channel. `psr_monitored_single` = single-channel PSR for cost-down builds. `hardwired_only` = mushroom button directly into engine-kill relay, no PSR (NOT recommended; refuses to boot if combined with `proportional_flow=true`). |
| `safety.estop_latency_ms_max` | int | `100` | `50..200` | W4-01 harness threshold | Per-build pass criterion. Tightening below 100 ms requires faster relays. |
| `safety.modbus_fail_latch_count` | int | `10` | `3..50` | `tractor_h7.ino`, `test_modbus_slave_sil.py` | Consecutive Modbus failures before `apply_control(-1)`. |
| `safety.m4_watchdog_ms` | int | `200` | `100..500` | `tractor_h7_m4.ino`, `test_m4_safety_sil.py` | M7→M4 alive-tick timeout. |

### Cameras

| id | type | default | range / enum | consumers | notes |
|---|---|---|---|---|---|
| `cameras.count` | int | `1` | `0..4` | `web_ui.py` (`_CAMERA_IDS` filtering), `tractor_x8` camera service, W4-08 harness | `0` = no camera at all (omit Coral, omit X8 camera service). UI hides the camera tile + the "Force keyframe" button. |
| `cameras.front_present` | bool | `true` | true/false | `web_ui.py` `_CAMERA_IDS` | Auto-coerced from `count`; explicit only when builder wires a single non-front cam. |
| `cameras.rear_present` | bool | `false` | true/false | same | Reverse-cam auto-select disabled when false. |
| `cameras.implement_present` | bool | `false` | true/false | same | |
| `cameras.crop_health_present` | bool | `false` | true/false | same, `topic 0x24` subscription | When false, base-station does NOT subscribe to the NDVI summary topic. |
| `cameras.coral_tpu` | enum | `mini_pcie` | `mini_pcie` \| `usb` \| `none` | `tractor_x8/inference.py`, `requirements.txt` | `none` falls back to CPU inference (slower, documented per-FPS budget). |

### Sensors (IMU / GPS)

| id | type | default | range / enum | consumers | notes |
|---|---|---|---|---|---|
| `sensors.imu_present` | bool | `true` | true/false | `tractor_x8` IMU service, `web_ui.py` tip-over warning, telemetry `pitch_deg`/`roll_deg` | When false, telemetry omits the fields entirely (does NOT publish `null`). |
| `sensors.imu_model` | enum | `bno086` | `bno086` \| `bno055` \| `icm20948` | IMU service driver selector | Determines which CircuitPython driver is loaded. |
| `sensors.gps_present` | bool | `true` | true/false | `tractor_x8` GPS service, `web_ui.py` map page, audit log lat/lon stamps | When false, the map page renders a "GPS not equipped" banner instead of an empty map. Audit-log lines omit lat/lon. |
| `sensors.gps_model` | enum | `neo_m9n` | `neo_m9n` \| `neo_m9n_rtk_f9p` | Driver selector | RTK upgrade path. |
| `sensors.hyd_pressure_sensor_count` | int | `2` | `0..2` | Opta `tractor_opta.ino` analog input handler | 0 = no pressure-sensor channels (telemetry omits `hyd_supply_psi`/`hyd_return_psi`). |

### Communication

| id | type | default | range / enum | consumers | notes |
|---|---|---|---|---|---|
| `comm.lora_region` | enum | `us915` | `us915` \| `eu868` \| `au915` | M7 `radio.begin`, MKR handheld | Sets the LoRa front-end + max-EIRP table. |
| `comm.cellular_backup_present` | bool | `false` | true/false | `tractor_x8` cellular service | When false, `tractor_x8` skips the SARA-R412M init and never publishes the cellular RSSI metric. |
| `comm.handheld_present` | bool | `true` | true/false | `lora_bridge.py` | When false, the bridge does not allocate a per-source replay window for the handheld. Operator-only base-station-driven builds. |

### Operator interface

| id | type | default | range / enum | consumers | notes |
|---|---|---|---|---|---|
| `ui.web_ui_enabled` | bool | `true` | true/false | `web_ui.py` startup | When false, the base-station container exits 0 immediately (headless deployments). |
| `ui.max_control_subscribers` | int | `4` | `1..32` | `web_ui.py` `_admit_ws` (Round 9 §B) | Per-build cap on concurrent `/ws/control` clients. |
| `ui.pin_required_for_control` | bool | `true` | true/false | `web_ui.py` PIN-gate | False permitted ONLY for closed-network demo builds. |

### Network

| id | type | default | range / enum | consumers | notes |
|---|---|---|---|---|---|
| `net.mqtt_host` | string | `localhost` | hostname | `web_ui.py`, `lora_bridge.py` (overridden by `LIFETRAC_MQTT_HOST` env per Round 25) | Per-build broker address. |
| `net.mqtt_port` | int | `1883` | `1..65535` | same | |

### Auxiliary attachment ports (Round 33 / BC-11)

| id | type | default | range / enum | consumers | notes |
|---|---|---|---|---|---|
| `aux.port_count` | int | `0` | `0..2` | M4 PWM init, `web_ui.py` aux-control surface | Number of aux hydraulic PWM channels physically wired. 0 = no aux ports installed (canonical default — most early v25 builds). |
| `aux.coupler_type` | string | `none` | `iso_5675` / `flat_face` / `none` | fleet docs, attachment-compat web hint | Quick-disconnect family. `iso_5675` is the legacy ag standard; `flat_face` is the skid-steer / construction standard. `none` when `port_count == 0`. |
| `aux.case_drain_present` | bool | `false` | true/false | M4 attachment-permit gate | Whether a third (case-drain) return line is plumbed. Required for motor-type attachments (auger, mower) with internal leakage; not required for cylinder-type attachments (grapple, thumb). |

## Out of scope (deliberately not in this schema)

* OTA delivery of build configs across the fleet (BC-XX out-of-scope item).
* Per-axis hydraulic flow auto-calibration (separate initiative; the
  schema only carries operator-supplied rated flows).
* Fundamentally different control fabrics (e.g. CAN bus instead of
  RS-485). The schema is for *presence/absence and parameters*, not for
  swapping the control fabric.

## Drift gate

The SIL test
[`base_station/tests/test_build_config_loader_sil.py`](base_station/tests/test_build_config_loader_sil.py)
class `BC_E_InventoryParity` parses this file's capability tables and
asserts that **every `id` listed above appears as a property in the
JSON Schema, and every property in the schema appears here.** Adding a
capability is therefore a three-place edit (this doc, the schema, the
default TOML); the test enforces no two of the three drift apart.
