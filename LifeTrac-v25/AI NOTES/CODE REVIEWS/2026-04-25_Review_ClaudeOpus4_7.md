# Code Review: LifeTrac v25 Comprehensive Pass
**Reviewer:** Claude Opus 4.7
**Date:** 2026-04-25

## Scope
This is a comprehensive review of the LifeTrac v25 control stack:

- [arduino_opta_controller/lifetrac_v25_controller.ino](../../arduino_opta_controller/lifetrac_v25_controller.ino) — Opta firmware (BLE + MQTT modes, deceleration, flow control)
- [esp32_remote_control/lifetrac_v25_remote.ino](../../esp32_remote_control/lifetrac_v25_remote.ino) — ESP32 dual-joystick remote
- [raspberry_pi_web_controller/app.py](../../raspberry_pi_web_controller/app.py) — Flask/SocketIO bridge to MQTT
- [raspberry_pi_web_controller/static/js/controller.js](../../raspberry_pi_web_controller/static/js/controller.js) — Browser controller UI
- [config/mosquitto.conf](../../config/mosquitto.conf) — Broker configuration
- Supporting configuration, install scripts, and READMEs

The mechanical OpenSCAD model and `ros2_bridge/` were not the focus of this pass; earlier reviews cover the mechanical design, and the ROS2 bridge mostly mirrors the MQTT contract reviewed here.

The previous review at [2026-04-16_Review_GPT5_4.md](2026-04-16_Review_GPT5_4.md) called out six findings. **All six are still present in the current code.** They are restated here at the same severity, with new evidence where the analysis can be deepened, and supplemented with additional findings discovered in this pass.

---

## Critical Findings

### 1. MQTT joystick values are coerced to integers, destroying proportional control
The Opta MQTT callback parses joystick fields with `| 0` defaults at [arduino_opta_controller/lifetrac_v25_controller.ino](../../arduino_opta_controller/lifetrac_v25_controller.ino#L329-L332). ArduinoJson infers the requested type from the default value, so `| 0` requests `int`, not `float`.

All publishers ([esp32_remote_control/lifetrac_v25_remote.ino](../../esp32_remote_control/lifetrac_v25_remote.ino#L222-L226) and [raspberry_pi_web_controller/app.py](../../raspberry_pi_web_controller/app.py#L116-L122)) emit floats in `[-1.0, 1.0]`. Truncation collapses the entire analog range to `{-1, 0, 1}` at the controller. The BLE path does not have this defect — only the MQTT path does — so MQTT users silently lose proportional control.

**Fix:** use `doc["left_x"] | 0.0f` (and the same for the other three axes). Optionally also clamp to `[-1.0, 1.0]` and reject NaN, mirroring `validateAndClampJoystickValue()` already used by BLE.

### 2. MQTT reconnect blocks the safety timeout — outputs latch on broker loss
In MQTT mode, the loop calls `reconnectMQTT()` before the timeout check at [arduino_opta_controller/lifetrac_v25_controller.ino](../../arduino_opta_controller/lifetrac_v25_controller.ino#L221-L226). `reconnectMQTT()` then blocks in `while (!client.connected())` with `delay(5000)` between attempts at [arduino_opta_controller/lifetrac_v25_controller.ino](../../arduino_opta_controller/lifetrac_v25_controller.ino#L283-L300).

Outputs are not driven low on disconnect, and `stopAllMovement()` is not invoked when the connection drops. If MQTT or WiFi fails while the machine is moving, the previously energized digital valves stay energized and the 4–20 mA flow output stays at its last current value until the broker comes back. The 1-second `SAFETY_TIMEOUT` is the documented fail-safe, but it cannot fire while the loop is parked inside `delay(5000)`.

**Fix:** call `stopAllMovement()` immediately on detected disconnect, make reconnect non-blocking (one attempt per loop with a `millis()`-gated retry), and evaluate the safety timeout on every loop iteration regardless of broker state.

### 3. Browser keyboard control latches the last command after key release
`updateKeyboardControl()` writes into `currentControl` on key press at [raspberry_pi_web_controller/static/js/controller.js](../../raspberry_pi_web_controller/static/js/controller.js#L170-L173), but the `activeKeys.size === 0` branch at [raspberry_pi_web_controller/static/js/controller.js](../../raspberry_pi_web_controller/static/js/controller.js#L177-L181) only refreshes the on-screen display — it never zeros `currentControl`.

The 50 ms command loop at [raspberry_pi_web_controller/static/js/controller.js](../../raspberry_pi_web_controller/static/js/controller.js#L312-L316) keeps publishing the stale values forever. Because each tick is a fresh non-zero command, the Opta's safety timeout never trips.

There is also a related stomp bug in the active branch at [raspberry_pi_web_controller/static/js/controller.js](../../raspberry_pi_web_controller/static/js/controller.js#L174-L186): whenever any key is held, `currentControl` is *replaced wholesale* with the keyboard values, overwriting any concurrent on-screen joystick input. Mixed keyboard + touch joystick control is silently broken.

**Fix:** in the no-keys branch, reset `currentControl.{left_x, left_y, right_x, right_y}` to zero. Better: use a separate `keyboardControl` accumulator (the variable already exists but is unused) and merge it with joystick output before sending.

### 4. ESP32 remote keeps publishing stale commands when a joystick disconnects
`readInputs()` only updates each joystick's axes when `joystick.connected()` returns true ([esp32_remote_control/lifetrac_v25_remote.ino](../../esp32_remote_control/lifetrac_v25_remote.ino#L177) and [esp32_remote_control/lifetrac_v25_remote.ino](../../esp32_remote_control/lifetrac_v25_remote.ino#L191)). If a Qwiic cable is unplugged or the joystick falls off the I²C bus mid-motion, the previously assigned `currentControl.left_*` / `right_*` values are left untouched. The transmit loop at [esp32_remote_control/lifetrac_v25_remote.ino](../../esp32_remote_control/lifetrac_v25_remote.ino#L113-L116) keeps publishing them.

The Opta then keeps moving until either (a) the user reconnects the joystick and physically centers it, or (b) the operator triggers an external stop. The Opta's safety timeout does not save us here because the remote is *still* sending commands; they are just stuck.

**Fix:** zero the corresponding axes when `joystick.connected()` is false. Optionally also drop publishing entirely if both joysticks are down, so the Opta safety timeout fires.

### 5. Web controller exposes hydraulic control to anyone reachable on the network
Flask-SocketIO is started with `cors_allowed_origins="*"` at [raspberry_pi_web_controller/app.py](../../raspberry_pi_web_controller/app.py#L41), and the `control_command` and `emergency_stop` handlers ([raspberry_pi_web_controller/app.py](../../raspberry_pi_web_controller/app.py#L243-L271) and [raspberry_pi_web_controller/app.py](../../raspberry_pi_web_controller/app.py#L274-L278)) accept input directly with no session, token, origin, or client identity check. The Flask `SECRET_KEY` is also a hardcoded literal at [raspberry_pi_web_controller/app.py](../../raspberry_pi_web_controller/app.py#L40).

Any browser tab on any device that can reach port 5000 can drive the machine. For a heavy hydraulic platform this is an operational safety issue, not just a generic OWASP A07 issue.

**Fix:** require login (even a single shared password gated through Flask-Login or HTTP basic auth behind TLS), restrict `cors_allowed_origins` to known origins, load `SECRET_KEY` from environment or `config.yaml`, and bind the server to a known interface or behind a reverse proxy with auth.

### 6. BLE control characteristics accept writes from any unpaired device
The BLE service is set up at [arduino_opta_controller/lifetrac_v25_controller.ino](../../arduino_opta_controller/lifetrac_v25_controller.ino#L621-L646) with characteristics declared as `BLERead | BLEWrite` and no encryption / pairing requirement. ArduinoBLE allows attribute-level encryption requirements (e.g. `BLEEncryption`) which are not used here.

In practice, any BLE central within radio range can scan, connect, write the joystick characteristics, and drive the machine. The DroidPad workflow does not require pairing either, so a hostile or accidental nearby device on the same UUIDs can take over.

**Fix:** require pairing/encryption on the joystick characteristics and add a connection-event hook that calls `stopAllMovement()` on disconnect (currently disconnect leaves the Opta in whatever state it was in until the safety timeout fires — which is OK, but explicit is better).

---

## High-Severity Findings

### 7. WiFi connect in `setup()` is an unbounded blocking loop
[arduino_opta_controller/lifetrac_v25_controller.ino](../../arduino_opta_controller/lifetrac_v25_controller.ino#L268-L276) busy-waits with `while (WiFi.status() != WL_CONNECTED) { delay(500); ... }`. If WiFi credentials are wrong or the AP is unreachable, the Opta never finishes `setup()`, never advertises BLE, and the operator sees only a blank LED — no fallback to BLE. The same pattern is used on the ESP32 at [esp32_remote_control/lifetrac_v25_remote.ino](../../esp32_remote_control/lifetrac_v25_remote.ino#L141-L147).

**Fix:** time-box `setupWiFi()` (e.g. 30 s), and on the Opta, fall back to BLE mode if MQTT mode WiFi fails to associate.

### 8. Mode switch and flow-valve jumper are read once at boot
`readModeSwitch()` and `readFlowValveConfig()` only run inside `setup()` ([arduino_opta_controller/lifetrac_v25_controller.ino](../../arduino_opta_controller/lifetrac_v25_controller.ino#L207-L217)). The README and [MODE_SWITCH_WIRING.md](../../MODE_SWITCH_WIRING.md) describe the switch as a runtime mode selector, but flipping it during operation has no effect until power-cycle. This is at minimum a documentation/behavior mismatch and at worst a safety surprise (operator believes they have switched the control source and they have not).

**Fix:** either (a) poll the switch in `loop()` and re-initialize on change while calling `stopAllMovement()`, or (b) update the documentation to state the switch is sampled only at boot.

### 9. Flask app `cleanup()` only runs on `KeyboardInterrupt`
[raspberry_pi_web_controller/app.py](../../raspberry_pi_web_controller/app.py#L385-L399) calls `cleanup()` only inside the `try/except KeyboardInterrupt/finally` of `__main__`. Under systemd (`lifetrac-web-controller.service`) the process receives `SIGTERM`, eventlet's `socketio.run` returns through a different path, and `camera_process` may be left orphaned. The MJPEG pipe holds the camera open and prevents the next start.

**Fix:** register `cleanup` with `atexit.register(cleanup)` and install a `signal.signal(signal.SIGTERM, ...)` handler that triggers shutdown.

### 10. MQTT credentials and broker IP are baked into firmware
`lifetrac` / `lifetrac_pass` and `192.168.1.100` are hardcoded in both the Opta firmware ([arduino_opta_controller/lifetrac_v25_controller.ino](../../arduino_opta_controller/lifetrac_v25_controller.ino#L48-L53)) and the ESP32 remote ([esp32_remote_control/lifetrac_v25_remote.ino](../../esp32_remote_control/lifetrac_v25_remote.ino#L26-L30)), and they are also the example credentials documented in the README. Any deployment that follows the README ships with a known username/password into a broker that, by [config/mosquitto.conf](../../config/mosquitto.conf) settings, has anonymous disabled but accepts these well-known credentials.

**Fix:** keep the example credentials in the README only as placeholders, and document that the deploying user must set unique credentials and broker IP. Consider a build-time `secrets.h` or NVS storage on the ESP32 / Opta so credentials are not committed.

### 11. Web app does not load the documented `config/config.yaml`
README and `.gitignore` ([raspberry_pi_web_controller/README.md](../../raspberry_pi_web_controller/README.md#L82) and [raspberry_pi_web_controller/.gitignore](../../raspberry_pi_web_controller/.gitignore#L17)) describe `config/config.yaml` as the runtime configuration source, but [raspberry_pi_web_controller/app.py](../../raspberry_pi_web_controller/app.py#L40-L54) hardcodes broker, port, credentials, camera resolution, and secret key. There is no YAML loader anywhere in `app.py`. Operators editing the documented file get no behavior change.

**Fix:** add a small YAML loader (PyYAML is already common) and override the module constants from it. Or remove the documentation that references the unused file.

### 12. Web controller default broker is wrong for the documented topology
[raspberry_pi_web_controller/app.py](../../raspberry_pi_web_controller/app.py#L42) hardcodes `MQTT_BROKER = "192.168.1.100"`, with the comment "Raspberry Pi's own IP". Since the broker runs *on the same Pi* per the install instructions, this should default to `127.0.0.1`. Mismatches between Pi DHCP lease and the literal `.100` will silently break MQTT publishing while the UI keeps reporting "connected" via SocketIO.

**Fix:** default to `127.0.0.1`; let `config.yaml` (when finding 11 is fixed) override.

---

## Medium-Severity Findings

### 13. Tank-steering math saturates instead of normalizing
`computeTrackSpeeds()` at [arduino_opta_controller/lifetrac_v25_controller.ino](../../arduino_opta_controller/lifetrac_v25_controller.ino#L342-L355) computes `left = baseSpeed + turnRate; right = baseSpeed - turnRate;` and then clamps to `[-1, 1]`. With full forward and full right (`baseSpeed = 1.0`, `turnRate = 1.0`), left clamps to `1.0` and right clamps to `0.0` — but with full forward and *half* right (`1.0 / 0.5`), left clamps from `1.5` to `1.0` while right is `0.5`. Operators perceive a "dead zone" near full forward where additional turn input has no effect on the outside track.

**Fix:** scale before clamp so the steering ratio is preserved: `if (max(|L|,|R|) > 1) { L /= m; R /= m; }`.

### 14. `processJoystickInput()` `previousInput = currentInput` is updated even during deceleration
At [arduino_opta_controller/lifetrac_v25_controller.ino](../../arduino_opta_controller/lifetrac_v25_controller.ino#L494) `previousInput = currentInput` is assigned at end of every call. This is fine for the active→zero transition detection, but if a new MQTT/BLE input arrives between deceleration ticks, `previousInput` will hold the *new* input on the next tick, and `handleAxisDeceleration` will not see the original active→zero edge. In practice MQTT/BLE update at 20 Hz and the loop runs at ~100 Hz, so this rarely surfaces, but a single timing nudge can cancel a deceleration ramp midway.

**Fix:** track the "armed" previous value separately from the in-flight target and update only when the deceleration completes or is cancelled.

### 15. `paho-mqtt` client is created with the deprecated v1 callback API
[raspberry_pi_web_controller/app.py](../../raspberry_pi_web_controller/app.py#L67) calls `mqtt.Client()` with no `callback_api_version` argument, so under `paho-mqtt >= 2.0` this issues a `DeprecationWarning` and the v1 callback shape will be removed in a future major. `requirements.txt` does not pin a major, so a routine reinstall can break callbacks.

**Fix:** pin `paho-mqtt < 2.0` *or* migrate to `mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, ...)` and update callback signatures.

### 16. SocketIO server has no rate limit and no per-client cap
A single misbehaving browser tab (or a deliberately abusive client, see finding 5) can flood `control_command` events. The handler at [raspberry_pi_web_controller/app.py](../../raspberry_pi_web_controller/app.py#L243-L271) processes and forwards every event synchronously to MQTT. There is no per-client throttle and no MQTT publish back-pressure handling.

**Fix:** debounce/coalesce control commands to one publish per ~25–50 ms per client, and reject events from a client that exceeds a sane max rate.

### 17. `visibilitychange` triggers `emergencyStop()` but `setInterval` may be throttled before the zeros land
At [raspberry_pi_web_controller/static/js/controller.js](../../raspberry_pi_web_controller/static/js/controller.js#L370-L376) the visibility handler calls `emergencyStop()`, which zeros `currentControl` and emits one `emergency_stop` event. Browsers throttle `setInterval` to ≥1 Hz when the tab is hidden. Combined with finding 3 (stale latches) this is mostly OK because `emergencyStop()` itself emits an immediate event, but if the event drops in transit, the next zero command may be a full second away. The Opta safety timeout (1 s) is exactly at the boundary.

**Fix:** in the `emergency_stop` handler on the server, also push a zero `control_command` to MQTT immediately (currently `send_control_command(0,0,0,0)` is called — this part is OK), and have the server publish a sticky "zero" every 100 ms for ~1 s after emergency stop to be robust to single-packet loss.

### 18. `delay(1000)` in `setup()` for "hardware stabilization"
[arduino_opta_controller/lifetrac_v25_controller.ino](../../arduino_opta_controller/lifetrac_v25_controller.ino#L203) blocks 1 s before reading the mode switch. It is not catastrophic but adds to first-boot latency. If the operator is relying on power-up motion suppression it actually helps, but it is undocumented why this delay is needed.

**Fix:** comment more thoroughly or replace with a `millis()`-gated read once the switch line has stabilized; a 50 ms delay is usually sufficient for a debounced pull-up.

### 19. `readBatteryVoltage()` uses `analogRead(A0)` with a placeholder formula
[esp32_remote_control/lifetrac_v25_remote.ino](../../esp32_remote_control/lifetrac_v25_remote.ino#L286-L293) returns a value calculated against an undocumented divider. It is published in `remote_status` and could mislead operators relying on "battery low" indication.

**Fix:** either remove the field entirely or document the divider and calibrate.

### 20. Camera MJPEG generator never reconnects on `libcamera-vid` exit
[raspberry_pi_web_controller/app.py](../../raspberry_pi_web_controller/app.py#L142-L196) breaks out of the read loop when `read()` returns empty, and the `finally` terminates the process. There is no restart logic; a single transient camera glitch leaves the video feed dead until the user reloads the page.

**Fix:** wrap in an outer retry loop with backoff, and emit a SocketIO event so the browser can show a "video lost" overlay instead of a frozen last frame.

---

## Low-Severity / Quality Findings

### 21. `serialEvent()` on ESP32 may not be invoked
[esp32_remote_control/lifetrac_v25_remote.ino](../../esp32_remote_control/lifetrac_v25_remote.ino#L353-L380) defines `serialEvent()`. The Arduino core for ESP32 historically does not call this — it is an AVR/SAMD convention. The "type `stop` to e-stop" feature documented in the file is dead code on ESP32.

**Fix:** poll `Serial.available()` from `loop()` directly.

### 22. JSON document sizes are static `DynamicJsonDocument(512)` / `(256)`
ArduinoJson 6 prefers `StaticJsonDocument` for fixed payloads or `JsonDocument` (v7). The current usage works but allocates from heap on every callback.

**Fix:** migrate to `StaticJsonDocument<256>` for these small messages, or upgrade to ArduinoJson 7.

### 23. Identical helper code duplicated between Opta and ESP32
Deadzone (`0.1`), the `[-1, 1]` clamp, the `* 2 / 2` joystick normalization in [esp32_remote_control/lifetrac_v25_remote.ino](../../esp32_remote_control/lifetrac_v25_remote.ino#L181-L187), and the JSON shape are repeated in three places. Drift is already starting (BLE clamps, MQTT does not). Consider a shared `lifetrac_protocol.h` consumed by both firmwares, or at least a comment in each file pointing to the canonical contract.

### 24. `controlValve()` and `controlTrack()` are functionally identical
Both at [arduino_opta_controller/lifetrac_v25_controller.ino](../../arduino_opta_controller/lifetrac_v25_controller.ino#L498-L527) implement "deadzone→both off, pos→fwd, neg→rev". They could be a single function. Minor.

### 25. `mqtt_test.py` and `calc_cut_coords.py` are unreviewed loose scripts
[test_scripts/mqtt_test.py](../../test_scripts/mqtt_test.py) is the only smoke test for the MQTT contract. It is not run as part of any CI (the existing [ARDUINO_CI.md](../../ARDUINO_CI.md) covers Arduino compile only). Adding a small pytest that publishes representative payloads and asserts the Opta firmware *would have* parsed them as floats (i.e. catches finding 1) would have caught the regression earlier.

### 26. Hardcoded `"YOUR_WIFI_SSID"` placeholders compile silently
Both firmwares accept the literal `"YOUR_WIFI_SSID"` and `"YOUR_WIFI_PASSWORD"` and just keep retrying WiFi connect (per finding 7). Add a `static_assert` or a runtime check that aborts to BLE-only mode when those literals are still present.

### 27. README and INSTALLATION_GUIDE describe runtime config that does not exist
Beyond finding 11, the [INSTALLATION_GUIDE.md](../../INSTALLATION_GUIDE.md) and several other markdown files reference behaviors (live mode switching, YAML config, etc.) that are aspirational. A doc-vs-code audit pass is overdue.

---

## Overall Assessment

The mechanical and hardware design work on v25 is mature and well-documented; the firmware/software side has a small number of repeating, high-impact patterns that need attention:

1. **Stale-command and fail-safe gaps** (findings 1, 2, 3, 4, 6, 8) dominate the risk profile. Each is independently capable of letting the machine continue moving past the operator's intent.
2. **Authentication is effectively absent** on every control surface (findings 5, 6, 10). A v25 deployed on a shop network is steerable by anyone on that network or any nearby BLE device.
3. **Documentation does not match runtime behavior** (findings 8, 11, 12, 26, 27). This is the kind of drift that quietly invalidates onboarding instructions.

None of the previous review's six findings have been addressed in code. Given that this is a heavy hydraulic platform, the recommendation is to land the safety-related fixes (1, 2, 3, 4, 8) before any new feature work.

---

## Recommended Fix Order

1. **Stop the latches.** Findings 1, 2, 3, 4. Together these eliminate the four ways the machine can keep moving after the operator stopped giving input.
2. **Decouple boot blocking from runtime safety.** Finding 7 (WiFi) and 8 (mode switch).
3. **Add an authentication boundary.** Findings 5, 6, 10 — at minimum a shared password and BLE pairing requirement.
4. **Make the documented config actually load.** Findings 11, 12 — and do a doc-vs-code sweep (finding 27) once that is done.
5. **Hygiene.** Findings 13–25.

A small CI test that round-trips a representative MQTT payload through the Opta JSON parser would catch the regression class behind finding 1 cheaply.
