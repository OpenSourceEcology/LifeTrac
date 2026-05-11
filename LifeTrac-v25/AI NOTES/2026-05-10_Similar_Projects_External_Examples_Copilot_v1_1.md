# 2026-05-10 Similar Projects External Examples (Copilot v1.1)

## Objective

Expand cross-project research beyond the initial v1.0 list with additional implementation-level examples that are directly relevant to LifeTrac Stage1 ingress work (Linux serial endpoint selection, GPIO reset ownership, startup sequencing, and repeatable host-side bring-up choreography).

## Scope Added In v1.1

This addendum focuses on concrete upstream implementations from:

1. Semtech legacy SX1301 gateway stack (`Lora-net/lora_gateway`)
2. Semtech SX1302 CoreCell stack (`Lora-net/sx1302_hal`)
3. ChirpStack concentratord docs and vendor model profiles (`chirpstack/chirpstack-concentratord`)
4. RAK gateway packaging layout (`RAKWireless/rak_common_for_gateway`)

## Additional Similar Projects and Concrete Examples

### 1) Semtech legacy gateway reset script (SX1301)

- Source:
  - https://raw.githubusercontent.com/Lora-net/lora_gateway/master/reset_lgw.sh
- Example details:
  - Script usage: `./reset_lgw.sh start|stop`
  - Default reset pin assignment: `IOT_SK_SX1301_RESET_PIN=7` (override via CLI arg)
  - Deterministic GPIO sequence:
    - export pin
    - set direction `out`
    - pulse reset `1 -> 0`
    - set direction `in`
    - unexport on stop
- Why it is similar:
  - This is a canonical Linux-side reset gate around a LoRa concentrator that mirrors our strict pre-probe reset discipline.

### 2) Semtech SX1302 HAL startup gating through reset helper script

- Sources:
  - https://raw.githubusercontent.com/Lora-net/sx1302_hal/master/readme.md
  - https://raw.githubusercontent.com/Lora-net/sx1302_hal/master/tools/reset_lgw.sh
- Example details:
  - `readme.md` states `tools/reset_lgw.sh` is called by every provided program that accesses SX1302 and must be colocated with executables.
  - `reset_lgw.sh` defines concrete control pins:
    - `SX1302_RESET_PIN=23`
    - `SX1302_POWER_EN_PIN=18`
    - `SX1261_RESET_PIN=22`
  - Script flow is explicit and deterministic:
    - `start`: `term -> init -> reset`
    - `stop`: `reset -> term`
- Why it is similar:
  - Explicit reset/power choreography before active traffic is treated as mandatory infrastructure, not optional recovery.

### 3) Semtech SX1302 tools invoke reset helper before chip access

- Source family:
  - `Lora-net/sx1302_hal` C utilities and packet forwarder code paths
- Example details:
  - Multiple utilities invoke:
    - `system("./reset_lgw.sh start")`
    - fail hard on non-zero return with reset-script diagnostics
- Why it is similar:
  - Reinforces a robust pattern for LifeTrac harness evolution: treat reset helper pass/fail as a first-class gate before probing protocol behavior.

### 4) ChirpStack concentratord config model: device paths + gpiochip pin ownership

- Sources:
  - https://www.chirpstack.io/docs/chirpstack-concentratord/configuration.html
  - https://raw.githubusercontent.com/chirpstack/chirpstack-concentratord/master/chirpstack-concentratord-sx1302/src/cmd/configfile.rs
- Example details:
  - Documented overridable mapping fields include:
    - `com_dev_path` (example: `/dev/spidev0.0`)
    - `sx1302_reset_chip` (example: `/dev/gpiochip0`)
    - `sx1302_reset_pin` (example: `17`)
    - `sx1302_power_en_chip` + `sx1302_power_en_pin`
    - `sx1261_reset_chip` + `sx1261_reset_pin`
- Why it is similar:
  - Exactly parallels our current challenge class: correct endpoint + correct ownership/control pin mapping are both required for reliable ingress.

### 5) ChirpStack vendor profile examples show board-specific reset/power pin defaults

- Sources:
  - https://raw.githubusercontent.com/chirpstack/chirpstack-concentratord/master/chirpstack-concentratord-sx1302/src/config/vendor/rak/rak2287.rs
  - https://raw.githubusercontent.com/chirpstack/chirpstack-concentratord/master/chirpstack-concentratord-sx1302/src/config/vendor/seeed/wm1302.rs
- Example details:
  - RAK2287 profile defaults include:
    - `com_path` default switches by USB flag (`/dev/ttyACM0` vs `/dev/spidev0.0`)
    - `sx1302_reset_pin` default via `/dev/gpiochip0`, pin `17`
  - Seeed WM1302 profile defaults include:
    - `com_path` default (`/dev/ttyACM0` or `/dev/spidev0.0`)
    - reset/power lines on `/dev/gpiochip0` with explicit pin defaults (reset, power enable, sx1261 reset)
- Why it is similar:
  - Demonstrates real-world coexistence of alternate communication transports plus board-specific reset/power ownership in production gateway software.

### 6) RAK common gateway packaging layout (profile-specific startup assets)

- Source:
  - https://github.com/RAKWireless/rak_common_for_gateway/tree/55fd13c12b/lora/rak2287
- Example details:
  - The profile tree includes:
    - `reset_lgw.sh`
    - `global_conf_i2c/`
    - `global_conf_uart/`
    - `global_conf_usb/`
    - gateway-forwarder sources/install scripts
- Why it is similar:
  - Confirms a mature operational pattern: one hardware profile, multiple transport/config variants, shared reset helper, and scriptable install/start paths.

## Distilled Cross-Project Pattern Updates (v1.1)

1. Deterministic reset helpers are commonly treated as required prerequisites before concentrator or modem traffic, not ad-hoc fixes.
2. Device-path selection and reset pin ownership are model-specific and explicitly configurable in mature stacks.
3. Production stacks frequently support multiple host transport paths (SPI, USB-UART), selected by runtime flags/profile.
4. Startup scripts and config profiles are structured so transport selection and reset/power sequencing can be switched without changing core protocol code.

## Direct Reuse Suggestions For LifeTrac

1. Promote reset/apply success to a hard precondition in the targeted harness (explicit pass/fail gate before probe attempt).
2. Keep endpoint and control-net ownership as independently testable dimensions, mirroring concentratord's split (`com_dev_path` vs reset/power mappings).
3. Introduce profile-style run manifests for candidate route-control ownership sets (analogous to vendor profile defaults), each with explicit endpoint + pin assumptions.
4. Continue immediate-repeat policy on every divergent signature to enforce reproducibility.

## Caveat

External examples are implementation references only. They do not supersede board-specific evidence under `DESIGN-CONTROLLER/bench-evidence/`; decisions remain evidence-gated on LifeTrac hardware.
