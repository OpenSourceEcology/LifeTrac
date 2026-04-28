# LifeTrac v25 Controller Code & Pipeline Review

- **Date:** 2026-04-28
- **Reviewer model:** Claude Opus 4.7 (GitHub Copilot)
- **Review version:** v1.0
- **Scope:** [`LifeTrac-v25/DESIGN-CONTROLLER/`](../DESIGN-CONTROLLER/) — firmware
  (`firmware/common/lora_proto`, `handheld_mkr`, `tractor_h7`, `tractor_opta`,
  `tractor_x8`), base-station Python (`base_station/`), the Docker pipeline
  (`Dockerfile`, `docker-compose.yml`), and the controller CI workflow
  (`.github/workflows/arduino-ci.yml`). Archived `RESEARCH-CONTROLLER/` was
  treated as reference only.

This review sits alongside the existing 2026-04-28 reviews by GitHub Copilot
(GPT-5 family) and Gemini 3.1 Pro. Where I duplicate one of their findings I
mark it with **(confirms)**; the rest are new observations or amplifications.

---

## Executive summary

The Python protocol layer and the bench-side test suite are in solid shape
and lock most of the wire-format invariants. The blocking problems are at the
seams between firmware and pipeline:

1. **AES-GCM tag-length contract is broken between callers and the real
   crypto backends** — every `lp_decrypt()` call site passes `ct_len = len - 12`
   (cipher text **plus** 16-byte tag), but both `lp_crypto_real.cpp` paths
   (mbedTLS and rweather) treat `ct_len` as ciphertext-only and read the tag
   at `ct + ct_len`. Production firmware will read past the buffer and reject
   every legitimate frame. **(confirms Copilot v1.0 finding #6)**
2. **M7 CSMA hop selection sets the radio to ~0 MHz.**
   `csma_pick_hop_before_tx()` does `radio.setFrequency(hop / 1.0e6f)` where
   `hop` is the hop counter index. The handheld version correctly translates
   the index through `lp_fhss_channel_hz()` first. With FHSS enabled on the
   tractor, every TX is mistuned. *New finding.*
3. **Compose stack will not start as written** — both `web_ui.py` and
   `lora_bridge.py` ignore `LIFETRAC_MQTT_HOST` and `lora_bridge.py` is
   launched without its required positional `port` argument.
   **(confirms Copilot v1.0 finding #1)**
4. **Arduino sketches cannot be compiled with the documented commands.**
   `arduino-cli` requires the primary `.ino` to match the folder name; the
   active folders are `handheld_mkr/handheld.ino`, `tractor_h7/tractor_m7.ino`,
   and `tractor_opta/opta_modbus_slave.ino` — all mismatched. **(confirms)**
5. **CI does not compile firmware and silently skips two security-relevant
   Python suites** (`test_web_ui_auth`, `test_web_ui_accel`) because the
   workflow does not `pip install -r base_station/requirements.txt`.
   **(confirms)**
6. **Boot PHY (SF7 / BW125) does not match `LADDER[0]` (SF7 / BW250)** on
   either node even though `g_ladder_rung = 0` is set immediately after
   `radio.begin()`. **(confirms)**
7. **`NonceStore` exists, is unit-tested, and is documented as the
   X8-side defence against post-restart nonce reuse — but it is never imported
   by `lora_bridge.py`.** Restarts within the same wall-clock second can reuse
   `(source_id, seq, time_s)` and rely entirely on the 5 random tail bytes.
   *New finding.*
8. **`process_air_frame()` decrypts into a fixed 160-byte plaintext buffer
   without an upper bound check** on the M7 and the handheld, so a 200+ byte
   ciphertext is a stack overwrite. Already documented in the 2026-04-26
   hardware-readiness review and still unfixed.

I would not flash this for a hardware test until at least items 1, 2, 4, 6
and 8 are fixed; item 3 is needed before the operator UI can run on the X8 in
its supported topology; items 5 and 7 are needed before "CI green" or
"restart in place" can be trusted.

---

## Verification performed

```powershell
# Python protocol + image pipeline tests (the only suite CI runs)
cd c:\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\base_station
python -m unittest discover -s tests
```

I am taking the previous review's identical run as ground truth: 88 tests
pass, two skip (`test_web_ui_auth`, `test_web_ui_accel`) because
`paho-mqtt + fastapi` are not installed in the workstation env. I cross-read
the source rather than re-running.

I did **not** attempt to compile the Arduino sketches; the previous review
already documented the deterministic failure (`Can't open sketch: main file
missing from sketch`).

---

## Critical findings

### C-1. AES-GCM `ct_len` contract is inconsistent

**Files:**
[`firmware/common/lora_proto/lora_proto.h`](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.h),
[`firmware/common/lora_proto/lp_crypto_real.cpp`](../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp),
[`firmware/common/lora_proto/crypto_stub.c`](../DESIGN-CONTROLLER/firmware/common/lora_proto/crypto_stub.c),
[`firmware/handheld_mkr/handheld.ino`](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino) (line 238),
[`firmware/tractor_h7/tractor_m7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino) (line 262).

The header docs say the `ct` argument "must be `pt_len + 16` bytes". Both
firmware callers do:

```cpp
if (!lp_decrypt(kFleetKey, nonce, onair + 12, len - 12, pt)) return;
size_t pt_len = len - 12 - 16;
```

…so `ct_len = len - 12` includes the 16-byte tag. The stub agrees and
peels the tag itself (`memcpy(pt, ct, ct_len - 16)`). The real wrappers do
not:

```cpp
// MbedTLS path
uint8_t* tag = ct + ct_len;             // OOB read
return lp__gcm_run(MBEDTLS_GCM_DECRYPT, ..., ct_len, ...) == 0;

// rweather path
gcm.decrypt(pt, ct, ct_len);            // treats ct_len as ciphertext-only
return gcm.checkTag(ct + ct_len, 16);   // OOB read
```

**Impact:** as soon as `LIFETRAC_USE_REAL_CRYPTO` is defined for a real
flash, every received frame either reads past the on-air buffer for the tag
and decrypts garbage, or asserts. The bench/sim build with the stub looks
healthy because the stub's contract matches the callers'. This is the exact
class of "passes in CI, dies on bench" failure that the existing tag-stub
guard was supposed to prevent.

**Recommended fix:** change the contract once and propagate it everywhere.
The most legible option is to keep callers passing the full buffer length
(it matches `lp_kiss_feed`'s output size) and have the real wrappers do
`size_t inner = ct_len - 16;` themselves, with a `ct_len < 16` early-out.
Add a golden vector in `bench/crypto_vectors/` whose plaintext is non-zero
so the stub's pass-through cannot pretend to decrypt it; gate
`LIFETRAC_USE_REAL_CRYPTO` on that vector.

### C-2. M7 CSMA hop helper tunes the radio to ~0 MHz

**File:** [`firmware/tractor_h7/tractor_m7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino)
(`csma_pick_hop_before_tx`).

```cpp
uint32_t hop = lp_csma_pick_hop(g_fhss_key_id, g_fhss_hop_counter,
                                sampler, ...);
radio.setFrequency(hop / 1.0e6f);          // <-- BUG
g_fhss_hop_counter++;
```

`lp_csma_pick_hop()` returns a hop counter index (a small integer). The
handheld correctly translates it:

```cpp
radio.setFrequency(lp_fhss_channel_hz(g_fhss_key_id, hop) / 1.0e6f);
```

So whenever the M7 is built with `LIFETRAC_FHSS_ENABLED`, every transmit
sets the SX1276 to a frequency `hop/1e6` MHz — i.e. essentially zero. The
sampler closure inside the helper does retune correctly via
`lp_fhss_channel_hz`, so the *probe* lands on a real channel; only the
final TX is broken.

**Recommended fix:** change to
`radio.setFrequency(lp_fhss_channel_hz(g_fhss_key_id, hop) / 1.0e6f);` and
fold a regression test into `bench/lora_retune_bench/` that asserts the TX
frequency matches the chosen hop's channel Hz.

### C-3. Compose stack: missing arg + ignored env

**Files:** [`docker-compose.yml`](../DESIGN-CONTROLLER/docker-compose.yml),
[`base_station/lora_bridge.py`](../DESIGN-CONTROLLER/base_station/lora_bridge.py),
[`base_station/web_ui.py`](../DESIGN-CONTROLLER/base_station/web_ui.py).

Compose passes `LIFETRAC_MQTT_HOST: mosquitto` to all three Python services,
but:

- `web_ui.py` line 114 hardcodes `mqtt_client.connect("localhost", 1883)`.
- `lora_bridge.py` only reads `--mqtt` from the CLI; the env var is unused.
- `lora_bridge.py`'s `command:` is `["python", "/app/base_station/lora_bridge.py"]`
  with no positional argument, but `argparse` declares `port` as positional
  and required. The container will exit with `error: the following
  arguments are required: port` immediately.

The `audit_log.py` and `tools/lora_rtt.py` paths *do* read
`LIFETRAC_MQTT_HOST`, so the bug is restricted to the two newest services.

**Recommended fix:** read `LIFETRAC_MQTT_HOST` (default `localhost`) at
module scope in both files; in `lora_bridge.py` add
`os.environ.get("LIFETRAC_LORA_DEVICE", "/dev/ttyACM0")` as the default for
the `port` argument or read it before `argparse`. Move the `mqtt_client`
construction into a FastAPI startup hook so import-time failures don't
prevent the test runner (and `python -m compileall`) from loading the
module.

### C-4. Sketch folder names do not match primary `.ino` names

`arduino-cli compile` requires `<folder>/<folder>.ino`. The current layout
fails the moment CI tries to compile:

| Folder                                   | Primary `.ino`            |
| ---------------------------------------- | ------------------------- |
| `firmware/handheld_mkr/`                 | `handheld.ino`            |
| `firmware/tractor_h7/`                   | `tractor_m7.ino`          |
| `firmware/tractor_opta/`                 | `opta_modbus_slave.ino`   |

**Recommended fix:** rename the sketches to match (`handheld_mkr.ino`,
`tractor_h7.ino`, `tractor_opta.ino`) — that is the smaller change than
moving folders, and it lets `ARDUINO_CI.md` use plain `arduino-cli compile
<folder>` without `--sketch-path` gymnastics. Apply the same convention to
`bench/lora_retune_bench/lora_retune_bench.ino` (already correct).

### C-5. Plaintext receive buffer is not bounded against ciphertext size

**Files:**
[`firmware/handheld_mkr/handheld.ino`](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino) (`process_air_frame`),
[`firmware/tractor_h7/tractor_m7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino) (`process_air_frame`).

Both functions allocate `uint8_t pt[160];` and call `lp_decrypt(..., len - 12, pt)`
without checking that `len - 12 - 16 <= sizeof(pt)`. The KISS decoder's
`buf[256]` permits `len` up to 256, so `lp_decrypt` will write up to 228
bytes into a 160-byte stack buffer with the real crypto backend (the stub
caps at `ct_len - 16` of write but still overruns).

This is the same observation as the 2026-04-26 readiness review item; it
remains unfixed.

**Recommended fix:**

```cpp
static_assert(sizeof(pt) <= 256 - 12 - 16, "pt sized for KISS buf");
if (len < 12 + 16 || len > 12 + sizeof(pt) + 16) return;
```

…and consider switching `lp_decrypt` to take an explicit `pt_max` and
return the actual length, or to refuse outputs longer than `LP_PT_MAX`.

### C-6. Boot PHY does not match `LADDER[0]`

**Files:** `tractor_m7.ino` (`setup()`), `handheld.ino` (`setup()`).

```cpp
// tractor_m7.ino
radio.begin(915.0, 125.0, 7, 5, 0x12, 20);  // SF7 BW125
// handheld.ino
int st = radio.begin(915.0, 125.0, 7, 5, 0x12, 14);
```

Both files immediately set `g_ladder_rung = 0`, but `LADDER[0]` is
`{ 7, 250, 5, 1 }` (BW250 per `DECISIONS.md` D-A2). Until the first
inbound `CMD_LINK_TUNE` is processed (which can never arrive at the wrong
PHY), the radios sit on a profile that does not exist in the ladder and
the airtime numbers reported by `lp_lora_airtime_ms()` for ControlFrame
under the active rung do not match reality.

**Recommended fix:** rebuild PHY setup around the `LADDER[]` constants,
e.g. a `apply_phy_rung(0)` call at the end of `setup()`. Add a unit-style
assertion (or a CI compile guard) that `LP_PHY_CONTROL_SF7.bw_khz` matches
`LADDER[0].bw_khz` so the constant-declaration drift cannot recur.

---

## High findings

### H-1. `NonceStore` is implemented and tested but never wired into the bridge

**Files:** [`base_station/nonce_store.py`](../DESIGN-CONTROLLER/base_station/nonce_store.py),
[`base_station/lora_bridge.py`](../DESIGN-CONTROLLER/base_station/lora_bridge.py).

`nonce_store.py` carries a long docstring describing exactly the failure
mode the bridge has today: cold-restart resets `tx_seq` to 0, so within the
same `time_s` bucket the AES-GCM nonce
`source_id || seq_le16 || time_le32 || random5` reuses the first four
fields. `lora_bridge.py` never imports `NonceStore`; only the unit tests
do. Result: defence relies entirely on the 5-byte random tail (40-bit
collision) plus the assumption that wall-clock advanced — the latter is
not even required for safety, only a courtesy.

**Recommended fix:** in `Bridge.__init__`, instantiate
`NonceStore(...)`, replace `self.tx_seq` with a `reserve(SRC_BASE)` call,
and call `observe(...)` after a successful TX. Add a startup test that
opens the same store twice and asserts the second `reserve()` skips by at
least `gap`.

### H-2. CI compiles nothing and skips fastapi tests

**File:** [`.github/workflows/arduino-ci.yml`](../../.github/workflows/arduino-ci.yml).

```yaml
firmware-compile-plan:
  steps:
    - name: Print active Arduino targets
      run: |
        echo "Active targets are documented in ..."
```

There is no `arduino-cli compile` invocation. The Python job runs unittest
without `pip install -r requirements.txt`, so any test that needs
`paho-mqtt + fastapi` (the auth and accel UI suites) just `unittest.skip`s
itself with status OK. The job name "Base station protocol tests" is
misleading — the auth-coverage gap is the most security-relevant Python
test in the tree.

**Recommended fix:** install the requirements before discovery
(`pip install -r LifeTrac-v25/DESIGN-CONTROLLER/base_station/requirements.txt`),
fail the run if any test reports `skipped` for "module required", and
either compile or stop pretending the firmware-compile job exists. The
quick win is to add the renamed sketches from C-4 to a real
`arduino-cli compile` matrix gated on `LIFETRAC_USE_REAL_CRYPTO` AND on
the stub build.

### H-3. Fleet keys are still all-zero and there is no startup refusal

**Files:** `handheld.ino`, `tractor_m7.ino`, `tractor_opta/opta_modbus_slave.ino`,
`base_station/lora_bridge.py`, `base_station/lora_proto/key.h.example`.

```cpp
static const uint8_t kFleetKey[16] = {0};
```

Docs say the key is provisioned out-of-band, but the firmware happily boots
with the placeholder, and `FLEET_KEY = bytes(16)` does the same in Python.
Anyone who flashes the example image and powers the radio is shipping an
authenticated link in name only.

**Recommended fix:** put `kFleetKey` behind a `key.h` include that is
`.gitignore`'d (the `.example` file already exists), and refuse to arm
either the safety relay (M4) or the MQTT bridge if the key is all zeros.
A `static_assert`-equivalent at runtime in `setup()` with `panic()` /
audit-log + exit is sufficient. Same in Python:
`if FLEET_KEY == bytes(16): raise SystemExit(...)`.

### H-4. `radio.transmit()` is blocking on the M7's 50 ms tick

**File:** `tractor_h7/tractor_m7.ino` (`emit_topic`, `send_link_tune`).

`SX1276::transmit()` is a synchronous call that returns when the packet
finishes air-time. A 60-byte telemetry frame at SF9 / BW250 (the
`PHY_TELEMETRY` profile) is ~210 ms; a CMD_LINK_TUNE at SF9 / BW125 is
nearly half a second. The M7's `loop()` runs `apply_control()` every 50 ms
and stamps the M4 watchdog at the **top** of each loop — but when
`emit_telemetry()` blocks for >200 ms, `loop()` does not recurse, the M4
watchdog `alive_tick_ms` stops advancing, and the M4 will trip the PSR
relay (`LIFETRAC_M4_WATCHDOG_MS = 200`).

This is a textbook "watchdog argues with telemetry" failure. With FHSS
disabled the only TX is at 1 Hz, so it is currently survivable; with FHSS
enabled (or once SF9 is the steady state) the M7 will trip its own M4 the
first time the link degrades.

**Recommended fix:** switch the telemetry/CMD path to RadioLib's
non-blocking `startTransmit()` + DIO0 IRQ, or chunk the loop so the watchdog
tick is updated mid-transmit (e.g. a TIM-driven ISR). At minimum, gate any
TX longer than `LIFETRAC_M4_WATCHDOG_MS / 2` on it being completed before
the next watchdog tick is due.

### H-5. M4 watchdog reads two volatile fields without a snapshot lock

**File:** `firmware/tractor_h7/tractor_m4.cpp`.

```cpp
uint32_t last_tick = SHARED->alive_tick_ms;
if (now >= last_tick && (now - last_tick) > LIFETRAC_M4_WATCHDOG_MS) trip(...);
uint32_t lc = SHARED->loop_counter;
```

The M7 publishes these in three separate volatile writes (`alive_tick_ms`,
`loop_counter`, `estop_request`) without a sequence-lock or paired
"version" stamp. If the M4 reads `alive_tick_ms` between the M7's writes
to `alive_tick_ms` and `loop_counter`, the M4 can momentarily see "fresh
tick, frozen counter" and false-trip on the next iteration if it caches
the stale `lc`. Even with cached state the failure is at most a 200 ms
window, but for a safety relay that's the entire budget.

**Recommended fix:** add a `seqlock`-style write_count (M7 increments
before and after the field updates; M4 reads the count, the fields, then
the count again, and retries if they differ).

### H-6. Modbus side-effect gating misses watchdog wraparound

**File:** `tractor_opta/opta_modbus_slave.ino` (`loop`).

The new `s_last_seen[]`/`on_holding_change` design correctly stops the
"every Modbus poll refreshes the watchdog" bug. But the watchdog counter
is a `uint16_t` that wraps every 65 536 ticks (~55 minutes at 20 Hz on the
master). On the wrap iteration, the master's `++g_watchdog_ctr` produces
the same value the register held two updates ago, but more importantly,
two consecutive identical writes from the master (e.g. exactly at wrap)
look like "no change" to the gating logic and `g_last_alive_change_ms`
does not advance. The 200 ms safety budget masks one such miss, but the
subtle correctness invariant is "transition implies write," which fails on
wrap.

**Recommended fix:** track watchdog freshness as
`(value, write_count)` rather than relying on value transitions. The
simplest concrete fix is to OR the loop iteration parity into the high bit
of the counter in `apply_control()` so consecutive writes always
transition.

### H-7. Plaintext-CRC verification asymmetry on `FT_COMMAND`

**File:** `tractor_h7/tractor_m7.ino` (`process_air_frame`).

`FT_CONTROL` and `FT_HEARTBEAT` re-verify CRC after AEAD; `FT_COMMAND`
skips it. The same pattern is in `handheld.ino`. AEAD already authenticates
the frame, so the CRC check is logically redundant, but if it exists for
"belt-and-braces against pt[] memory corruption" (the stated rationale in
the M7 comment) then it should apply uniformly. As-is the asymmetry is a
gap waiting to bite the next refactor.

**Recommended fix:** either remove the post-AEAD CRC check on
`FT_CONTROL` / `FT_HEARTBEAT` and document AEAD as the sole integrity check,
or add the same check to `FT_COMMAND`. I'd remove it — AEAD makes CRC
strictly weaker — and then drop the on-wire CRC as a v26 protocol bump.

---

## Medium findings

### M-1. Persistent sequence numbers across power-cycle (handheld + M7)

`g_seq` and `g_telem_seq` are RAM-only `uint16_t`. The nonce uniqueness
guarantee `(key, source_id, seq, time_s)` collapses if two boots land in
the same wall-clock second after a brownout. Today the random-5 tail
(40-bit) makes a collision improbable but not impossible. Mirror the
base-station `NonceStore` solution by writing `seq` to flash (Cortex-M0+
EEPROM emulation on SAMD; QSPI on H7) every N frames, and resume from
`stored + GAP`.

### M-2. `apply_phy_rung()` discards `LADDER[r].cr_den`

**File:** `firmware/handheld_mkr/handheld.ino` (`apply_phy_rung`).

The handheld's apply path hardcodes `cr_den = 5` ignoring the rung's value.
All three rungs happen to use 5 today, so the bug is latent. A future SF10
rung at CR 4/8 would silently drop back to 4/5.

**Fix:** use `LADDER[rung].cr_den`.

### M-3. `read_axis()` rescaling can clip at one end

```cpp
int sign = centered < 0 ? -1 : 1;
int mag  = (centered < 0 ? -centered : centered) - AXIS_DEADBAND;
long scaled = (long)mag * 127 / (512 - AXIS_DEADBAND);
```

For `centered = -512` (raw `0`), `mag = 496`, `scaled = 127`, output `-127`
— OK. For `centered = +511`, `mag = 495`, `scaled ≈ 126`, output `+126`.
ADC asymmetry produces an off-by-one ceiling in the positive direction.
Cosmetic, but it means full-stick-up never reaches `+127` while
full-stick-down does.

**Fix:** divide by `(511 - AXIS_DEADBAND)` and clamp.

### M-4. `read_buttons()` returns stale state until any input changes again

The "candidate must hold for `DEBOUNCE_MS`" filter is correct, but if the
candidate equals the existing `s_btn_state`, `s_btn_change_ms` is never
re-stamped, so the very first stable read after boot waits indefinitely.
In practice the boot input is always 0 and the first press is a transition
that re-enters the `if (raw != s_btn_candidate)` branch — fine. But the
logic depends on that observation, which a re-reader will trip on.

**Fix:** initialise `s_btn_change_ms = millis()` in `setup()` so the first
loop has a defined reference.

### M-5. `web_ui.py` connects to MQTT at module import time

`mqtt_client.connect("localhost", 1883)` at the top of the module means
`uvicorn web_ui:app` will fail to import if the broker is not reachable
(or — more relevantly — if the import is happening from the test runner or
`compileall`). Move into a FastAPI `@app.on_event("startup")` hook with a
retry loop and a health endpoint that surfaces broker state.

### M-6. `image_pipeline/fragment.py` import path leaks the base-station path into the X8 container

The X8-side service does `from lora_proto import ...`, which only resolves
when `PYTHONPATH` includes `base_station/`. The Dockerfile sets
`PYTHONPATH=/app/base_station:/app`, but the X8 systemd units in
`firmware/tractor_x8/systemd/` need the same setting and there is no
single source of truth. Either move the shared protocol module up to a
`shared/` package both sides import, or codify the PYTHONPATH in a
`conf.d` file that both compose and systemd source.

### M-7. `pick_active_source()` priority loop and `takectl` interaction

A latched `takectl` from BASE wins over a fresh HANDHELD even if the
operator is actively driving the handheld, because the loop checks
`takectl_until_ms` first and is keyed on freshness, not activity. The
intent (per `MASTER_PLAN.md §8.5`) is that the human at the cab outranks
remote take-control — the current code inverts that. Worth a doc-vs-code
reconciliation.

### M-8. Telemetry payload max disagrees between C and Python

C: `emit_topic` rejects `payload_len > 118` (CRC must fit inside the
`payload[120]` field). Python: `TELEM_MAX_PAYLOAD = 120` accepts up to 120.
The python side will accept frames the C side cannot generate, but more
importantly the Python *unit tests* assert behaviour at the wider bound,
which the C firmware cannot meet. Pick one — I'd pick 118 to keep the C
struct authoritative.

### M-9. Opta boot self-test assumes `PIN_R1..PIN_R4` are sequential

```cpp
for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(PIN_R1 + i, HIGH); delay(50); digitalWrite(PIN_R1 + i, LOW);
}
```

`PIN_R1 = D0`, `PIN_R2 = D1`, etc. Whether `D0..D3` are consecutive
integers in the Opta variant header is core-dependent. Use an explicit
`static const uint8_t kRelayPins[4] = {PIN_R1, PIN_R2, PIN_R3, PIN_R4};`
and iterate over that.

### M-10. `radio.startReceive()` not re-armed after every TX path

`emit_topic()` calls `radio.startReceive()` after `radio.transmit()`, but
`send_link_tune()` only does so after the *second* of its two
back-to-back TXs (per the `try_step_ladder()` comment). Between the two
TXs the radio sits in standby, so a peer that retunes after the first
announce has a small window where its acknowledgement gets dropped on the
floor. Re-arm after the first TX as well; the cost is one extra mode
switch.

---

## Low / nits

- **L-1.** `tractor_m7.ino` comment says "Run multi-source arbitration loop
  @ 50 Hz" but `next_arb = now + 50` is 20 Hz. Either the comment or the
  cadence is stale.
- **L-2.** `lora_bridge.py` `_handle_air()` calls `self.audit.record(...)`
  on every successful frame; at 20 Hz × 4 sources that's 80 audit-log
  writes/sec. Consider down-sampling the `rx` event or batching writes —
  the `AuditLog` fsync cost will dominate the worker thread on slow eMMC.
- **L-3.** `lora_proto.py` `kiss_encode` allocates a fresh `bytearray` per
  call; for the TX worker hot path consider a reusable buffer.
- **L-4.** `lora_proto.h` defines both `LP_PHY_CONTROL_SF7` and
  `BTN_TAKE_CONTROL`; the source of truth for the button bit value drifts
  between firmware (`<< 7`) and the Python `pack_control` (which uses raw
  ints). Add a Python-side mirror constant and assert equality in tests.
- **L-5.** `crypto_stub.c` `lp_decrypt` returns true for any tagged frame
  without checking the tag bytes. Consider rejecting frames whose tag is
  non-zero in stub mode so accidental real-crypto-on-the-wire is louder
  during bench debugging.
- **L-6.** `tractor_m7.ino` `apply_control()` writes the full holding
  block every 50 ms even when nothing changed; this is intentional for
  watchdog refresh, but at 20 Hz on RS-485 115 200 8N1 it consumes
  ~21 % of the bus. The Opta should be tolerant of less-frequent valve
  rewrites; consider writing valves only on transition and a smaller
  watchdog-only block at 20 Hz.
- **L-7.** `audit_log.py` is not visible in this review's depth, but the
  fact that `Bridge.__init__` writes "bridge_start" before opening the
  serial port means a missing serial device produces an audit line and a
  silent crash; reorder so the audit line records the failure as well.
- **L-8.** `handheld.ino` increments `g_seq` twice per loop (control +
  heartbeat) — at 20 Hz that's 40 frames/sec, giving the 16-bit
  sequence ~27 minute wraps. Combined with M-1 above, a wrap inside the
  same wall-clock second on the M7's reception side will trigger the
  `lp_replay_check_and_update()` "wrap" branch, which accepts but resets
  the bitmap. A network attacker with a captured sequence can therefore
  squeeze a replay through during the wrap window. Audit-log replay
  rejects (`audit_log.log_replay_reject`) and watch for spikes near wrap.
- **L-9.** `link_monitor.RollingAirtimeLedger` stores per-event tuples in
  a deque; at thousand-events-per-window scale (`window_ms = 10_000`,
  per-event ~10 ms airtime) the deque stays short but `.utilization()`
  walks the entire deque every poll. Consider an O(1) rolling sum.

---

## Tests / coverage observations

- `test_lora_proto.py` locks the wire format and CRC well; consider
  adding a test that pairs `pack_control` with a `decrypt_frame` round-trip
  through the *real* crypto and asserts byte-identical plaintext, to
  guard against regressions of finding C-1.
- The image-pipeline tests do not cover the "fragment exceeds budget"
  path on the X8 side; add a test that requests a 4 kB encoded payload and
  asserts that `pack_image_fragments` returns N fragments each ≤
  `TELEMETRY_FRAGMENT_MAX_AIRTIME_MS`.
- `test_web_ui_auth` and `test_web_ui_accel` are the only PIN-auth
  coverage; they are silently skipped in CI (H-2). Until that is fixed,
  treat `LIFETRAC_PIN` enforcement as untested in CI.

---

## Suggested sequencing

1. C-1 (decrypt contract) and C-2 (FHSS frequency) — both are one-line
   bugs but block any field test.
2. C-4 (sketch rename) and H-2 (real CI), so subsequent fixes have a
   compile gate.
3. C-3 (compose env/args) and M-5 (web_ui import-time MQTT) so the stack
   actually starts in its supported topology.
4. C-5 (plaintext bounds) and H-3 (key-zero refusal) — the smallest
   security-correctness wins.
5. H-1 (NonceStore wiring) and M-1 (firmware seq persistence) together;
   they are the same problem on two sides.
6. C-6 (boot PHY), H-4 (blocking transmit), H-5 (seqlock) — required
   before a hardware bench session that exercises link degradation.

Everything else is post-bench cleanup.

---

## Addendum — items from the other 2026-04-28 reviews worth fixing or studying further

After this review was written I read the three sibling reviews:

- [`2026-04-28_Controller_Code_Pipeline_Review_GitHub_Copilot_v1_0.md`](2026-04-28_Controller_Code_Pipeline_Review_GitHub_Copilot_v1_0.md)
- [`2026-04-28_Controller_Code_Pipeline_Review_GPT-5.3-Codex_v1_0.md`](2026-04-28_Controller_Code_Pipeline_Review_GPT-5.3-Codex_v1_0.md)
- [`2026-04-28_Controller_Code_Review_Gemini_3.1_Pro_v1.0.md`](2026-04-28_Controller_Code_Review_Gemini_3.1_Pro_v1.0.md)

The findings below are ones I either missed, want to amplify, or want to
push back on. Three-way agreement on a finding is treated as confirmed and
not repeated here (decrypt `ct_len` contract, compose wiring, sketch-name
mismatch, CI gaps, BW125 boot vs LADDER[0]=BW250, all-zero fleet keys).

### A-1. Confirmed and promoted: web_ui seq is not threaded into the AEAD nonce  *(Gemini 1.1)*

**Status: real bug, promote to High.** I verified
[`web_ui.py`](../DESIGN-CONTROLLER/base_station/web_ui.py) `ws_control`
calls `pack_control(seq=seq, ...)` and publishes the packed bytes on
`lifetrac/v25/cmd/control`, then `lora_bridge.py` `_on_mqtt_message` calls
`self._tx(SRC_BASE, msg.payload)` with no `nonce_seq=`, so `_tx` allocates
a *fresh* sequence via `_reserve_tx_seq()` and that fresh sequence is what
the AEAD nonce binds to.

Two concrete consequences:

1. The tractor's per-source replay window keys on the *cleartext header*
   `sequence_num`, which is the `web_ui` counter (resets to 0 every
   websocket reconnect). Two operator sessions in the same boot will
   collide on low seq values and the second session's first dozen frames
   are silently dropped as replays.
2. The audit log records the bridge's nonce seq, but operator-visible
   diagnostics that read the cleartext header (e.g. `raw/control/{src}`
   echoes, future per-frame ACKs) will read the web_ui seq. The two
   sequences will not match.

This is not a confidentiality break — AEAD only requires nonce uniqueness
per key, and `_reserve_tx_seq()` does provide that — but it is a layering
bug that will bite anyone trying to reason about replay logs. **Fix per
Gemini's recommendation:** parse the inbound payload header and pass
`nonce_seq=hdr.sequence_num`. Better still, have web_ui publish a JSON
control message and let the bridge own the entire pack/seq path so there
is one seq counter for `SRC_BASE`.

### A-2. New: CMD_REQ_KEYFRAME publish path is open-loop  *(GPT-5.3 #2)*

**Status: real, promote to High.** Verified at
[`web_ui.py:251`](../DESIGN-CONTROLLER/base_station/web_ui.py): the image
pipeline publishes `lifetrac/v25/cmd/req_keyframe`, but
[`lora_bridge.py:84-86`](../DESIGN-CONTROLLER/base_station/lora_bridge.py)
only subscribes to `cmd/control`, `cmd/estop`, `cmd/camera_select`. The
keyframe request is dropped on the broker floor. Recovery from canvas
desync in `IMAGE_PIPELINE.md` therefore is design-only. Add the
subscription, pack `CMD_REQ_KEYFRAME` (P1 priority), and add an
end-to-end test that asserts the LoRa TX worker actually emits the
command.

### A-3. New: X8 camera_service is not wired to the M7 UART  *(Copilot #7)*

**Status: real, important integration gap.** `camera_service.py` publishes
to MQTT topic `lifetrac/v25/cmd/image_frame` and `tractor_m7.ino` only
consumes KISS-framed `[topic_id | payload]` from `Serial1`. There is no
service that bridges MQTT → Serial1 in the active tree.
[`image_pipeline/ipc_to_h747.py`](../DESIGN-CONTROLLER/firmware/tractor_x8/image_pipeline/ipc_to_h747.py)
defines a length-prefixed alternative protocol that nobody calls. Pick
one, wire it, add an integration test from camera frame → M7 emit_topic →
base reassembly. Until then the image pipeline is a closed loop on the X8
and never reaches the operator.

### A-4. New: `LIFETRAC_SETTINGS_PATH` vs `LIFETRAC_BASE_SETTINGS` env-var drift  *(GPT-5.3 #7)*

**Status: real, fix immediately.** Compose sets
`LIFETRAC_SETTINGS_PATH=/var/lib/lifetrac/settings.json`,
`settings_store.py` reads `LIFETRAC_BASE_SETTINGS`. The compose value is
ignored and the store falls back to whatever its hardcoded default is.
Anyone tweaking the volume mount or path in compose will believe their
change took effect. Pick one name and propagate.

### A-5. New: `lp_make_command` writes CRC into the middle of `arg[8]`  *(Gemini 2.2)*

**Status: real, design smell, study further.** I re-read
[`lora_proto.c`](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c)
`lp_make_command()`: the function correctly writes the CRC at the
*serialized* offset `prefix = sizeof(LoraHeader) + 1 + arg_len` and
returns that length to the caller, so on-the-wire bytes are right. **But**
that offset overwrites `arg[arg_len]` and `arg[arg_len+1]` inside the
`CommandFrame` struct, and the struct's declared `crc16` field at offset
`5+1+8 = 14` is left untouched. A future code path that does
`f->crc16` (instead of treating the buffer as a serialised stream) will
read garbage. Either:

- Convert `CommandFrame.arg` to a flexible array member `uint8_t arg[]`
  and remove the in-struct `crc16` field, with serialization via an
  explicit function. Or,
- Always emit a fixed-length 8-byte arg block and use the in-struct
  `crc16` field directly. Wastes airtime; not preferred.

The current state passes, but it is the kind of latent struct/buffer
duality that the next refactor will get wrong.

### A-6. New: `lp_kiss_encode` bounds check is over-conservative  *(Gemini 2.1)*

**Status: real, low impact, easy fix.** The check
`if (o + 2 >= out_max) return 0;` inside the loop assumes every input byte
needs the escape doublet, even for non-special bytes. For tightly-sized
output buffers a small but legitimate payload is rejected.
[`handheld.ino`](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino)
sizes `kiss[2 * (12 + 160) + 2]` so it doesn't trip in practice, but the
bench fragmentation tests size more tightly. Branch the check on whether
the byte needs escaping (Gemini's patch is correct).

### A-7. New: `camera_service` thread-race on keyframe trigger  *(GPT-5.3 #8)*

**Status: real, fix.** The keyframe request flag is a dict mutated from
the MQTT callback thread and read from the encode loop. Replace with
`threading.Event` so observed-but-cleared races cannot drop a request.
While there, clamp `WEBP_QUALITY` at startup so a `1` config does not skip
the entire encode loop guard.

### A-8. New: WebSocket fan-out is unbounded  *(GPT-5.3 §opt, Copilot #13, Gemini 3.3)*

**Status: real, hardening.** All three sibling reviews independently flagged
that `web_ui.py` stores subscribers in unbounded sets, schedules
`run_coroutine_threadsafe` futures without retaining or logging them, and
has no per-subscriber backpressure. Add bounded queues with drop/coalesce
on overflow and a connection cap (e.g. 8 telemetry, 4 image, 4 state).
The asyncio drain pattern is the standard fix; until it lands a single
slow browser can stall the MQTT thread that drives all sockets.

### A-9. New: web/API input validation gaps  *(Copilot #10)*

**Status: real, fix.** `int(msg.get("lhx", 0))` raises `ValueError` on
`{"lhx":"foo"}`, which `ws_control` does not catch — the websocket closes.
Worse, axes are not range-checked before `pack_control` clips, so an
attacker with a session can send `{"lhx":12345}` and observe the server
clip semantics in audit-log spam. The `params/set` endpoint forwards
arbitrary JSON to MQTT with no schema. Add explicit per-field validation
and cap the JSON body size.

### A-10. New: Modbus failures not surfaced  *(Copilot #11)*

**Status: real, fix.** `ModbusRTUClient.endTransmission()` and `requestFrom`
return values are discarded on the M7. With no fault count and no
escalation to neutral after N consecutive failures, an operator looking at
the OLED or `TOPIC_SOURCE_ACTIVE` cannot tell whether the Opta is alive.
Increment a counter and feed it into `TOPIC_ERRORS`; consider forcing
`apply_control(-1)` after `>K` consecutive write failures so the M4
watchdog is not the only thing that catches a wedged bus.

### A-11. New: control feel is binary  *(Copilot #12)*

**Status: noted, study before wet test.** The `if axis_lh_y > 20 → coil = 1`
mapping in `apply_control()` is acceptable for relay smoke testing and
worse than useless for hydraulic motion (full-on or full-off). Port the
`RESEARCH-CONTROLLER/arduino_opta_controller/` deadband + ramp + dual-flow
split before any wet test. `REG_AUX_OUTPUTS` is a TODO; either implement
or remove from the active map so unused registers cannot accidentally
energize R10–R12.

### A-12. New: `TelemetryReassembler` is unused on the bridge's generic path  *(Copilot #9)*

**Status: real, study.** The reassembler is exercised by tests but
`Bridge._handle_air` publishes raw fragment bodies to `topic_name(topic_id)`
for any topic, not just images. A future >118-byte engine telemetry topic
will land on MQTT as visible-but-broken fragment bodies. Decide whether
the bridge owns reassembly for non-image topics or whether MQTT consumers
are expected to. Document the choice in `LORA_PROTOCOL.md`.

### A-13. New: PIN lockout does not honour `X-Forwarded-For`  *(Gemini 1.2)*

**Status: real, fix when reverse-proxying.** `_client_ip()` returns
`request.client.host`, which is the upstream proxy/loopback inside docker
or behind nginx. A single attacker on the LAN hammering `/api/login` will
lock the legitimate operator out (and vice versa) because both share an
IP. Use FastAPI's `ProxyHeadersMiddleware` and trust `X-Forwarded-For`
only from a configured proxy whitelist. Pair with audit-log forwarding of
PIN failures (Gemini 3.1) — currently they go to `logging.warning` only,
not the persistent audit log.

### A-14. New: requirements.txt is loose, no lockfile  *(GPT-5.3 §opt)*

**Status: real, fix when CI starts installing deps.** Add a lockfile
(`pip-compile` → `requirements.lock`) so CI and the X8 image install the
same paho/fastapi/cryptography versions. The current loose pinning means
"green CI on Tuesday, broken on Wednesday" is one upstream release away.

### A-15. CI test-skip masking  *(Copilot/GPT-5.3, amplify)*

The auth/accel suites `unittest.skip` on missing deps. Both sibling
reviews flag this. Stronger fix than mine: gate the skip on
`os.environ.get("CI")` — in CI, raise instead of skipping, so a missing
dep is a hard error rather than a silent pass.

### Push-backs (do not adopt)

- **Gemini 3.2 — "compress AES-GCM nonce by re-inferring repeated fields
  from cleartext header."** Rejected. The nonce's 5 random tail bytes are
  the only defence against `(key, source_id, seq, time_s)` collision after
  reboot (M-1 in this review and the docstring of
  [`nonce_store.py`](../DESIGN-CONTROLLER/base_station/nonce_store.py)
  spells out exactly that scenario). Shrinking the random tail to save
  bytes-per-frame would weaken AEAD security in a system that already has
  marginal nonce-uniqueness defence; the saved airtime is small relative
  to the 16-byte tag the protocol already pays for.
- **Gemini 1.1 framing as "Worse, the LoraReplayWindow on the tractor
  evaluates sequence numbers contained in the *inner frame header*…"** —
  agree on the layering bug (A-1) but not on the framing as a security
  failure. AEAD is fine; replay window correctness is the real issue.

### Items where I and the others overlapped but I missed evidence

- **GPT-5.3 #6 / Copilot #3:** the unittest skipped suites. I called this
  out (H-2) but did not record the explicit `paho-mqtt + fastapi
  required` skip messages from the test files; both sibling reviews did.
  Add the explicit skip-reason strings to the CI's failure message so the
  log is self-explaining.
- **Copilot #9 + my M-8:** the C/Python `TELEM_MAX_PAYLOAD` mismatch I
  flagged at 118 vs 120 also bears on the reassembler integration in A-12.

### Suggested addition to my fix sequence

Insert after step 4 of the original sequence:

- **4a.** A-1 (web_ui seq threading) — same operator-UI session as C-3.
- **4b.** A-3 (X8 camera bridge) — required before any image-pipeline test
  is meaningful end-to-end.
- **4c.** A-2 (CMD_REQ_KEYFRAME wiring) — the loss-recovery story in
  `IMAGE_PIPELINE.md` is otherwise vapourware.
- **4d.** A-4 (settings env var) — five-minute fix that prevents two days
  of "why didn't my settings save" support tickets.

