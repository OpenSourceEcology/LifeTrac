# LifeTrac v25 Controller — Code-Review Implementation Plan

**Date:** 2026-04-28
**Author:** Claude Opus 4.7 (synthesis pass)
**Source reviews:**

- [2026-04-28_Controller_Code_Pipeline_Review_ClaudeOpus4_7_v1_0.md](2026-04-28_Controller_Code_Pipeline_Review_ClaudeOpus4_7_v1_0.md)
  (primary, includes the cross-review addendum)
- [2026-04-28_Controller_Code_Pipeline_Review_GitHub_Copilot_v1_0.md](2026-04-28_Controller_Code_Pipeline_Review_GitHub_Copilot_v1_0.md)
- [2026-04-28_Controller_Code_Pipeline_Review_GPT-5.3-Codex_v1_0.md](2026-04-28_Controller_Code_Pipeline_Review_GPT-5.3-Codex_v1_0.md)
- [2026-04-28_Controller_Code_Review_Gemini_3.1_Pro_v1.0.md](2026-04-28_Controller_Code_Review_Gemini_3.1_Pro_v1.0.md)

This document is the actionable union of all four reviews. Each work item
has a stable ID (`IP-###`), an originating finding, the file(s) to touch,
the concrete change, the acceptance test, and the wave it belongs to. IDs
are stable so the TODO checklist can reference them without ambiguity.

ID prefix `IP` = "Implementation Plan". Severity tags `[BLOCKER]`,
`[HIGH]`, `[MED]`, `[LOW]`, `[CHORE]`.

---

## Wave 0 — repo hygiene needed before anyone can build

These items make the rest of the plan testable. Do them first; none
require hardware.

### IP-001 [BLOCKER] Rename sketch folders / `.ino` files to match Arduino IDE rules
*Findings:* Claude C-4 / Copilot #2 / Gemini 5.1.

Arduino IDE requires `<folder>/<folder>.ino`. Today
[`firmware/handheld_mkr/handheld.ino`](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino)
and
[`firmware/tractor_h7/tractor_m7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino)
will not open or compile from the IDE.

**Change:**
- Rename folder `firmware/handheld_mkr/` → `firmware/handheld_mkr/` is
  fine but the file must be `handheld_mkr.ino`. Rename `handheld.ino` →
  `handheld_mkr.ino` (or rename folder to `handheld/`).
- Same for `tractor_h7/tractor_m7.ino` → either folder `tractor_m7/` or
  file `tractor_h7.ino`. Recommend `tractor_h7/tractor_h7.ino` and keep
  `tractor_m4.cpp` as-is.
- Update every doc and CI reference (`arduino-cli compile` paths,
  README, BUILD-CONTROLLER guides, M7/M4 split docs).

**Acceptance:** `arduino-cli compile -b arduino:samd:mkrwan1310 firmware/handheld_mkr/`
succeeds locally and in CI.

### IP-002 [BLOCKER] Wire `LIFETRAC_PIN`, `LIFETRAC_MQTT_HOST`, `LIFETRAC_LORA_DEVICE` through compose → containers
*Findings:* Claude C-3 / Copilot #1 / GPT-5.3 #1 / Gemini 4.

The web_ui and lora_bridge containers need three things to even start:
the operator PIN, the broker hostname, and the LoRa serial device. None
are passed in `docker-compose.yml`.

**Change to [`base_station/docker-compose.yml`](../DESIGN-CONTROLLER/base_station/docker-compose.yml):**

```yaml
services:
  lora_bridge:
    environment:
      - LIFETRAC_MQTT_HOST=mosquitto
      - LIFETRAC_LORA_DEVICE=/dev/ttyACM0
      - LIFETRAC_FLEET_KEY_FILE=/run/secrets/fleet_key
    devices:
      - "/dev/ttyACM0:/dev/ttyACM0"
    command: ["python", "-m", "lora_bridge", "${LIFETRAC_LORA_DEVICE:-/dev/ttyACM0}"]

  web_ui:
    environment:
      - LIFETRAC_MQTT_HOST=mosquitto
      - LIFETRAC_PIN_FILE=/run/secrets/operator_pin
      - LIFETRAC_SETTINGS_PATH=/var/lib/lifetrac/settings.json
    secrets:
      - operator_pin
      - fleet_key

secrets:
  operator_pin:
    file: ./secrets/operator_pin
  fleet_key:
    file: ./secrets/fleet_key
```

Read the `_FILE` variants in code (standard Docker secrets pattern). See
also IP-005 (env-var name unification) and IP-008 (key loading).

**Acceptance:**
- `docker compose config` validates.
- `docker compose up` starts all four services with no `KeyError`.
- `pytest base_station/tests` still green.

### IP-003 [BLOCKER] Fix `lp_decrypt(ct_len)` contract mismatch
*Findings:* Claude C-1 / Copilot #4 / GPT-5.3 #4 / Gemini 2.4.

Real backend (`lp_crypto_real.cpp`) treats `ct_len` as ciphertext-only and
reads tag at `ct + ct_len`. Stub (`crypto_stub.c`) treats `ct_len` as
ciphertext-plus-tag and copies `ct_len - 16`. Callers
([`handheld.ino:238`](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino),
[`tractor_m7.ino:262`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino))
pass `len - 12`, which means swapping backends silently changes wire
semantics by 16 bytes.

**Change:** Pick one contract — recommend "ct_len = ciphertext-only,
tag follows". Update:

- `crypto_stub.c::lp_decrypt` to read tag at `ct + ct_len` and return
  plaintext length `ct_len`.
- All call sites to pass `len - 12 - 16` for ct_len, with explicit named
  constants `LP_NONCE_LEN = 12`, `LP_TAG_LEN = 16`.
- Add the contract to the docstring of `lp_decrypt` in
  [`lp_crypto.h`](../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto.h).

**Acceptance:** Add a unit test `tests/test_lp_decrypt_contract.py` that
encrypts with the Python AESGCM, calls each C backend through ctypes, and
asserts plaintext bytes match. Run against both stub and real builds.

### IP-004 [BLOCKER] Fix M7 CSMA frequency bug
*Finding:* Claude C-2 / GPT-5.3 #5.

In
[`tractor_m7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino),
`csma_pick_hop_before_tx()` calls
`radio.setFrequency(hop / 1.0e6f)` where `hop` is the channel *index*,
not Hz. Without FHSS this evaluates to ~0 Hz.

**Change:** Replace with
`radio.setFrequency(lp_fhss_channel_hz(g_fhss_key_id, hop) / 1.0e6f);`.
Mirror the handheld helper exactly — extract a shared inline in
[`firmware/common/lora_proto/lp_phy.h`](../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_phy.h)
so the two TX paths cannot drift again.

**Acceptance:** Bench test: handheld → tractor link works with
`-DLIFETRAC_FHSS_ENABLED`. Add a static-analysis lint (grep CI step) that
fails if `setFrequency(.*hop.*)` appears outside the helper.

### IP-005 [BLOCKER] Resolve `LIFETRAC_SETTINGS_PATH` vs `LIFETRAC_BASE_SETTINGS` env-var drift
*Findings:* Claude addendum A-4 / GPT-5.3 #7.

[`settings_store.py`](../DESIGN-CONTROLLER/base_station/settings_store.py)
reads `LIFETRAC_BASE_SETTINGS`; compose (when fixed in IP-002) sets
`LIFETRAC_SETTINGS_PATH`. Pick `LIFETRAC_SETTINGS_PATH` (matches `_FILE`
suffix convention used elsewhere) and update the store.

**Acceptance:** Unit test `test_settings_store_env_var.py` reads both old
and new names with a one-release deprecation warning on the old.

### IP-006 [BLOCKER] Match boot PHY to `LADDER[0]`
*Findings:* Claude C-6 / Copilot #6 / GPT-5.3 #5 / Gemini 2.6.

Both sketches call `radio.begin(915.0, 125.0, 7, 5, 0x12, 20)` (BW125)
but `LADDER[0] = {SF7, BW250, CR5, hop=1}`. First TX after boot uses the
wrong PHY, which the receiver rejects until a `CMD_LINK_TUNE` arrives —
chicken-and-egg.

**Change:** Helper `lp_apply_ladder_rung(radio, &LADDER[0])` called from
both sketches' `setup()`. Unit test asserts `LADDER[0]` matches the
documented control PHY in `LORA_PROTOCOL.md`.

**Acceptance:** Two-board bench: cold boot both ends, first frame
decodes.

### IP-007 [BLOCKER] CI must build firmware and run all suites
*Findings:* Claude H-2 / Copilot #3 / GPT-5.3 #6 / Gemini 5.2.

Today CI skips the auth/accel suites with `unittest.skip` when
`paho-mqtt`/`fastapi`/`cryptography` are missing, and does not invoke
`arduino-cli` at all.

**Change to [`.github/workflows/ci.yml`](../../.github/workflows/ci.yml)
(or wherever the CI lives):**

- Install all base_station requirements + dev requirements before tests.
- Add `arduino-cli core install arduino:samd arduino:mbed_portenta`.
- Add `arduino-cli lib install` from
  [`arduino_libraries.txt`](../DESIGN-CONTROLLER/firmware/arduino_libraries.txt).
- Compile both sketches with and without
  `-DLIFETRAC_USE_REAL_CRYPTO`.
- In CI, gate test skips on `os.environ.get("CI") != "true"` — in CI a
  missing dep is a hard error, not a skip (Claude addendum A-15).

**Acceptance:** PRs cannot merge with red CI; `pytest --no-skip` reports
zero skips on the CI runner.

### IP-008 [BLOCKER] Replace zero placeholder fleet keys with provisioned-or-fail loading
*Findings:* Claude H-3 / Copilot #5 / GPT-5.3 #3 / Gemini 4.

Both
[`firmware/common/lora_proto/lp_keys.h`](../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_keys.h)
and
[`base_station/lora_bridge.py:70`](../DESIGN-CONTROLLER/base_station/lora_bridge.py)
ship a `bytes(16)` placeholder. Anyone flashing the example image is
running an authenticated link in name only.

**Change:**
- Move `kFleetKey` out of the header into a separate
  `lp_keys_secret.h` that is `.gitignore`'d, with a
  `lp_keys_secret.example.h` showing the format.
- Add a build-time check: if `LIFETRAC_USE_REAL_CRYPTO` is defined and
  `kFleetKey` is all-zero, `static_assert(false, ...)`. Or, runtime
  check in `setup()` that halts with OLED message
  "FLEET KEY NOT PROVISIONED".
- Python: `FLEET_KEY = _load_key_or_die(os.environ["LIFETRAC_FLEET_KEY_FILE"])`
  — refuse to start if the file is missing or all zero.
- Update
  [`KEY_ROTATION.md`](../DESIGN-CONTROLLER/KEY_ROTATION.md) with the
  new file paths and bootstrap sequence.

**Acceptance:** Fresh checkout cannot accidentally talk to a real
tractor; integration test flips a single byte and asserts decryption
failure.

---

## Wave 1 — protocol / pipeline correctness

These are the High-severity items that the system design depends on.
None should block on hardware.

### IP-101 [HIGH] Wire `NonceStore` into the LoRa bridge
*Finding:* Claude H-1.

[`base_station/nonce_store.py`](../DESIGN-CONTROLLER/base_station/nonce_store.py)
exists, has tests, and is never imported by `lora_bridge.py`. Without
it, a base-station restart can replay sequence numbers within the same
`time_s`/random window before the random tail saves us.

**Change:** Bridge constructor opens
`NonceStore(path=os.environ.get("LIFETRAC_NONCE_STORE", "/var/lib/lifetrac/nonce.db"))`,
calls `reserve()` before TX and `observe()` after RX of every encrypted
frame. Persistence file added to compose volume.

**Acceptance:** Existing `tests/test_nonce_store.py` integrated into a
new `tests/test_lora_bridge_nonce.py` that simulates restart + replay.

### IP-102 [HIGH] Thread `nonce_seq` from web_ui through the bridge
*Finding:* Claude addendum A-1 / Gemini 1.1.

`web_ui.ws_control` packs `pack_control(seq=seq, ...)` with its own
counter, then `lora_bridge._on_mqtt_message` calls
`self._tx(SRC_BASE, msg.payload)` with no `nonce_seq=`, so a *fresh*
seq is reserved for the AEAD nonce. Inner header seq and outer nonce
seq diverge.

**Change (preferred):** Move all `pack_control` / `pack_camera_select`
construction into the bridge. Web_ui publishes a small JSON message
(`{"axes":[...], "buttons":[...], "flags":...}`); bridge owns the only
seq counter for `SRC_BASE`. Removes the duplicate-counter category of
bug entirely.

**Alternative (lower-blast-radius):** Bridge parses inbound payload
header and passes `nonce_seq=hdr.sequence_num` into `_tx`.

**Acceptance:** New integration test publishes 100 control messages,
asserts AEAD nonce seq == cleartext header seq for every frame, and
asserts no replay-window false-rejects.

### IP-103 [HIGH] Subscribe and forward `cmd/req_keyframe`
*Finding:* Claude addendum A-2 / GPT-5.3 #2.

Web_ui publishes `lifetrac/v25/cmd/req_keyframe`; bridge does not
subscribe. The "loss-recovery" story in `IMAGE_PIPELINE.md` is
non-functional.

**Change:** Add `cmd/req_keyframe` to `MQTT_SUB_TOPICS`; handler packs
`CMD_REQ_KEYFRAME` (P1 priority) and emits via `_tx`. Add round-trip
integration test.

### IP-104 [HIGH] Wire X8 camera_service to the M7 UART
*Finding:* Claude addendum A-3 / Copilot #7.

Today `camera_service.py` publishes encoded frames to MQTT
`lifetrac/v25/cmd/image_frame`; M7 reads only KISS over `Serial1`.
Nothing connects them.

**Change:** Either:

- **Option A (recommended):** Decommission the MQTT publish; have
  `camera_service.py` write KISS frames directly to `/dev/ttymxc1` (X8
  UART to H7), using the topic-prefix scheme already in
  `LORA_PROTOCOL.md`. Adopt
  [`image_pipeline/ipc_to_h747.py`](../DESIGN-CONTROLLER/firmware/tractor_x8/image_pipeline/ipc_to_h747.py)'s
  length framing, or document the KISS choice and delete the unused
  alternative.
- **Option B:** Add a small mqtt-to-serial bridge service.

**Acceptance:** End-to-end test: synthetic camera frame in →
`StatePublisher` MQTT image fragments out.

### IP-105 [HIGH] M4 read of shared SRAM4 must be seqlock-protected
*Finding:* Claude H-5.

[`tractor_m4.cpp`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m4.cpp)
reads `SHARED->alive_tick_ms` and `SHARED->loop_counter` without a
seqlock. On a torn read of `alive_tick_ms` the M4 may declare M7 dead
and trip E-stop falsely.

**Change:** Bump version of `SharedM7M4` and add a `uint32_t seq;`
field. M7 writes `seq++; ...; seq++;` (odd = mid-write). M4 reads
`seq, payload, seq2`; retries if `seq` differs or is odd. Add a unit
test that runs both sides under thread-sanitizer-equivalent
single-process simulation.

### IP-106 [HIGH] Watchdog wrap-around on M4
*Finding:* Claude H-6.

`if (now - SHARED->alive_tick_ms > 200)` is fine for `uint32_t`
arithmetic, but the M4 trips on first `SHARED->estop_request != 0`
which has no debounce — a single torn read of `1` is enough.

**Change:** Require `estop_request` to be `0xA5A5A5A5` (magic) for the
M4 to honor it; any other non-zero value is treated as torn read and
ignored.

### IP-107 [HIGH] M7 LoRa TX is blocking against the M4 watchdog
*Finding:* Claude H-4.

`radio.transmit()` blocks the M7 thread; if the M7 is mid-TX when the
M4 is about to evaluate `alive_tick_ms`, the M4 may trip. SF9/BW125
TXes can take >150 ms.

**Change:** Replace blocking transmit with `startTransmit()` /
`isTransmitDone()` polling driven by `lora_proto`. Update
`alive_tick_ms` from a high-priority hardware timer ISR rather than
the main loop.

### IP-108 [HIGH] Fix CRC asymmetry on `FT_COMMAND` (`lp_make_command` struct/buffer duality)
*Findings:* Claude H-7 / Claude addendum A-5 / Gemini 2.2.

`lp_make_command` writes the CRC at byte offset
`prefix = sizeof(LoraHeader) + 1 + arg_len`; for `arg_len < 8` this
lands inside `arg[8]` and the struct's declared `crc16` field is left
stale. Future code reading `f->crc16` will read garbage.

**Change:** Convert `CommandFrame.arg` to flexible array
`uint8_t arg[]`, drop the in-struct `crc16`, and use only the
serialized buffer. Add a fuzz test that round-trips
`lp_make_command` → `lp_parse_command` for `arg_len ∈ [0, 8]`.

---

## Wave 2 — base-station service hardening

### IP-201 [MED] Move `mqtt_client.connect("localhost", 1883)` into a startup hook
*Finding:* Claude M-5.

Module-import-time connect breaks `compileall`, breaks the test runner,
and silently falls back when the broker name should come from env.

**Change:** FastAPI `@app.on_event("startup")` with retry/backoff. Use
`os.environ["LIFETRAC_MQTT_HOST"]`. Add `/healthz` exposing broker
state.

### IP-202 [MED] Bound WebSocket fan-out and per-subscriber queues
*Findings:* Claude addendum A-8 / Copilot #13 / GPT-5.3 §opt / Gemini 3.3.

Subscriber sets are unbounded; `asyncio.run_coroutine_threadsafe`
futures are discarded; one slow browser stalls the MQTT thread.

**Change:** Bounded queues per subscriber with drop/coalesce on
overflow; per-class connection cap (8 telemetry, 4 image, 4 state);
remove sockets on send failure; log dropped frame counts.

### IP-203 [MED] Validate WebSocket and `/api/params` inputs
*Finding:* Claude addendum A-9 / Copilot #10.

`int(msg.get("lhx", 0))` only catches `JSONDecodeError`; ValueError /
TypeError closes the socket. `/api/params` accepts arbitrary JSON.

**Change:** Pydantic models for both surfaces; per-field range checks
(axes ±127, buttons u16, flags u8); JSON-body size cap on `/api/params`;
explicit parameter whitelist.

### IP-204 [MED] Decode telemetry through `TelemetryReassembler` on the bridge
*Finding:* Claude addendum A-12 / Copilot #9.

`Bridge._handle_air` publishes raw fragment bodies to
`topic_name(topic_id)` for every topic, not just images. Future
>118-byte telemetry topics will land on MQTT broken.

**Change:** Bridge reassembles for non-image topics too; document the
choice in `LORA_PROTOCOL.md`. Image fragments stay raw because the
web_ui builds the canvas incrementally.

### IP-205 [MED] Surface Modbus failures
*Finding:* Claude addendum A-10 / Copilot #11.

`ModbusRTUClient.endTransmission()` and `requestFrom` returns are
discarded on the M7. Operators cannot tell if Opta is alive.

**Change:** Counter incremented on every fault; surface via
`TOPIC_ERRORS`; `apply_control(-1)` after K consecutive failures so
the M4 watchdog is not the only safety net.

### IP-206 [MED] Honor `X-Forwarded-For` for PIN lockout
*Finding:* Claude addendum A-13 / Gemini 1.2.

`_client_ip()` returns `request.client.host`. Behind nginx or in
docker, all browsers collapse to one IP — first attacker locks out
the operator.

**Change:** Adopt FastAPI `ProxyHeadersMiddleware` with a
`LIFETRAC_TRUSTED_PROXIES` env var; only honor `X-Forwarded-For` from
the configured proxy(s). Pair with IP-207.

### IP-207 [MED] Forward auth failures to `AuditLog`
*Finding:* Gemini 3.1.

PIN failures go to `logging.warning`; survive only as long as the
container's stderr buffer.

**Change:** Add `audit.event("auth_fail", ip=..., reason=...)` calls
alongside the existing log lines. Surfaces in `/audit` exports.

### IP-208 [MED] Settle the `camera_service.py` thread race + clamp `WEBP_QUALITY`
*Finding:* Claude addendum A-7 / GPT-5.3 #8.

Keyframe-trigger dict mutated from MQTT thread, read from encode loop;
`WEBP_QUALITY = 1` would skip the encode-loop guard.

**Change:** Replace dict with `threading.Event`; clamp
`WEBP_QUALITY = max(20, min(100, …))` at startup with a warning.

### IP-209 [MED] Lockfile the Python deps
*Finding:* Claude addendum A-14 / GPT-5.3 §opt.

`requirements.txt` is loose; CI greens then breaks on next upstream
release.

**Change:** `pip-compile requirements.in → requirements.lock`; CI uses
the lock; X8 systemd image uses the lock; document refresh procedure.

---

## Wave 3 — firmware polish

### IP-301 [MED] Initialize `s_btn_change_ms`
*Finding:* Claude M-4.

`read_buttons()` button-debounce reference is undefined at boot.

**Change:** `s_btn_change_ms = millis()` in `setup()`.

### IP-302 [MED] Fix axis dead-band scaling
*Finding:* Claude M-3.

Scales by `511` instead of `(511 - AXIS_DEADBAND)`, leaving a
percentage of axis travel unreachable.

**Change:** Divide by `(511 - AXIS_DEADBAND)` and clamp to `[-127, 127]`.

### IP-303 [MED] Implement `REG_AUX_OUTPUTS` or remove it
*Findings:* Claude M-9 / Copilot #12.

`REG_AUX_OUTPUTS` is TODO; valve mapping is binary.

**Change:** Either implement aux-output handling on the Opta or remove
the register from the published map so unused registers cannot
accidentally energize R10–R12. Pair with porting the
`RESEARCH-CONTROLLER/arduino_opta_controller/` deadband + ramp before
any wet hydraulic test (Wave 4 prerequisite).

### IP-304 [MED] Move X8 services to a stable PYTHONPATH
*Finding:* Claude M-6.

`from lora_proto import ...` only resolves with the bridge's
`PYTHONPATH`. The X8 systemd units do not set it.

**Change:** Either install `lora_proto` as a proper package
(`setup.cfg`) or set `Environment=PYTHONPATH=/opt/lifetrac/base_station`
in the systemd units.

### IP-305 [LOW] `lp_kiss_encode` over-conservative bounds check
*Finding:* Claude addendum A-6 / Gemini 2.1.

Branch the bounds check on whether the byte needs escaping (Gemini's
patch). Adds a unit test for tight-buffer encoding.

### IP-306 [LOW] Reconcile `TELEM_MAX_PAYLOAD` between C and Python
*Finding:* Claude M-8 / Copilot #9.

C says 118; Python says 120. Pick one (recommend 118 to match the
PHY-image rung's 32-byte fragment math) and document it.

### IP-307 [LOW] Per-source-active OLED status threshold
*Finding:* Claude M-10.

Boundary condition on the "no source active" indicator.

### IP-308 [LOW] Boot self-test should not assume sequential PIN ordering
*Finding:* Claude M-7.

`PIN_R1..PIN_R4` happen to be sequential today; future board revs
won't be. Iterate over an explicit array.

### IP-309 [LOW] Camera tile-diff numpy vectorization
*Finding:* GPT-5.3 §opt.

Optional perf win on X8.

---

## Wave 4 — pre-wet-test gates

The system **must not** drive hydraulics until these are done:

1. IP-001 through IP-008 (Wave 0).
2. IP-101, IP-102, IP-104, IP-105, IP-106, IP-107, IP-108 (Wave 1).
3. IP-303 implemented (not just stubbed) **plus** the
   `RESEARCH-CONTROLLER/arduino_opta_controller/` ramp/deadband port.
4. Bench test: handheld E-stop latches the tractor within 100 ms across
   100 retries.
5. Bench test: link-tune walk-down through all PHY rungs without packet
   loss > 1%.
6. Bench test: M7-M4 watchdog trip on simulated M7 hang.

---

## Items explicitly **not** adopted (with reasoning)

### NA-1 — Compress AES-GCM nonce by inferring repeated fields
*Source:* Gemini 3.2.

The 5 random tail bytes are the only defence against
`(key, source_id, seq, time_s)` collision after reboot — exactly the
post-restart scenario `nonce_store.py` exists to mitigate. Shrinking
the random tail to save bytes-per-frame would weaken AEAD security.

### NA-2 — Frame Claude-A-1 / Gemini 1.1 as a confidentiality bug
The AEAD bound is fine; this is a layering / replay-window correctness
issue (IP-102), not an encryption break.

---

## Cross-reference index

| ID | Severity | Origin | Files |
|---|---|---|---|
| IP-001 | BLOCKER | Claude C-4, Copilot #2, Gemini 5.1 | `firmware/handheld_mkr/`, `firmware/tractor_h7/` |
| IP-002 | BLOCKER | Claude C-3, Copilot #1, GPT-5.3 #1, Gemini 4 | `base_station/docker-compose.yml` |
| IP-003 | BLOCKER | Claude C-1, Copilot #4, GPT-5.3 #4, Gemini 2.4 | `firmware/common/lora_proto/lp_crypto*.{c,cpp,h}`, `handheld.ino`, `tractor_m7.ino` |
| IP-004 | BLOCKER | Claude C-2, GPT-5.3 #5 | `tractor_m7.ino`, `firmware/common/lora_proto/lp_phy.h` |
| IP-005 | BLOCKER | Claude addendum A-4, GPT-5.3 #7 | `base_station/settings_store.py`, compose |
| IP-006 | BLOCKER | Claude C-6, Copilot #6, GPT-5.3 #5, Gemini 2.6 | both `.ino` setup() |
| IP-007 | BLOCKER | Claude H-2, Copilot #3, GPT-5.3 #6, Gemini 5.2 | `.github/workflows/ci.yml` |
| IP-008 | BLOCKER | Claude H-3, Copilot #5, GPT-5.3 #3, Gemini 4 | `lp_keys.h`, `lora_bridge.py` |
| IP-101 | HIGH | Claude H-1 | `lora_bridge.py`, `nonce_store.py` |
| IP-102 | HIGH | Claude addendum A-1, Gemini 1.1 | `web_ui.py`, `lora_bridge.py` |
| IP-103 | HIGH | Claude addendum A-2, GPT-5.3 #2 | `lora_bridge.py` |
| IP-104 | HIGH | Claude addendum A-3, Copilot #7 | `firmware/tractor_x8/camera_service.py`, `tractor_m7.ino` |
| IP-105 | HIGH | Claude H-5 | `tractor_m4.cpp`, shared SRAM4 header |
| IP-106 | HIGH | Claude H-6 | `tractor_m4.cpp` |
| IP-107 | HIGH | Claude H-4 | `tractor_m7.ino` |
| IP-108 | HIGH | Claude H-7, addendum A-5, Gemini 2.2 | `firmware/common/lora_proto/lora_proto.{c,h}` |
| IP-201 | MED | Claude M-5 | `web_ui.py` |
| IP-202 | MED | Claude addendum A-8, Copilot #13, GPT-5.3 §opt, Gemini 3.3 | `web_ui.py` |
| IP-203 | MED | Claude addendum A-9, Copilot #10 | `web_ui.py` |
| IP-204 | MED | Claude addendum A-12, Copilot #9 | `lora_bridge.py` |
| IP-205 | MED | Claude addendum A-10, Copilot #11 | `tractor_m7.ino` |
| IP-206 | MED | Claude addendum A-13, Gemini 1.2 | `web_ui.py` |
| IP-207 | MED | Gemini 3.1 | `web_ui.py`, `audit_log.py` |
| IP-208 | MED | Claude addendum A-7, GPT-5.3 #8 | `firmware/tractor_x8/camera_service.py` |
| IP-209 | MED | Claude addendum A-14, GPT-5.3 §opt | `base_station/requirements*.{txt,in,lock}` |
| IP-301 | MED | Claude M-4 | `handheld.ino` |
| IP-302 | MED | Claude M-3 | `handheld.ino` |
| IP-303 | MED | Claude M-9, Copilot #12 | `opta_modbus_slave.ino`, `tractor_m7.ino` |
| IP-304 | MED | Claude M-6 | X8 systemd units |
| IP-305 | LOW | Claude addendum A-6, Gemini 2.1 | `lora_proto.c` |
| IP-306 | LOW | Claude M-8, Copilot #9 | `lora_proto.h`, `lora_proto.py` |
| IP-307 | LOW | Claude M-10 | `handheld.ino` |
| IP-308 | LOW | Claude M-7 | `opta_modbus_slave.ino` |
| IP-309 | LOW | GPT-5.3 §opt | `camera_service.py` |
