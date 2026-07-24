# LoRa TX/RX Optimization — Post-Implementation Review

**Date:** 2026-07-23
**Author:** GitHub Copilot (Claude Fable 5) — author of the original review/plan, closing the loop
**Version:** v1.0
**Reviews:** commits `968505ae` ("Harden image LoRa TX/RX fragmentation path") and `476e41d2` ("feat(lora): LoRa Communication & Image TX/RX Pipeline Optimization"), the uncommitted working-tree edits on both daemons, the untracked `run_live_radio_monitor.ps1`, and the claims in [`2026-07-23_LoRa_Comm_and_Image_TX_RX_Optimization_Implementation_and_Hardware_Verification_Copilot_v1_0.md`](./2026-07-23_LoRa_Comm_and_Image_TX_RX_Optimization_Implementation_and_Hardware_Verification_Copilot_v1_0.md) (the "implementation doc").
**Method:** every claimed change diffed against git (`1e6fa9b9..476e41d2` + working tree); the touched Python test modules re-run **by this reviewer** in isolation and in combination; the new C target `check-tx-len-guard` built and run by this reviewer (PASS); the hardware log excerpts re-derived against the airtime model and the committed counter logic.

---

## 1. Executive summary

The implementation delivers the **shape** of the Phase-0/Phase-1 plan faithfully — pacing, sizing, contract check, self-heal, redundancy, stale-drop, tests, Makefile CI fix all exist in the tree, and several details *improve* on the plan's sketches. The live 2-board bench run is real and its headline number (388.0 B/s, 2.15× baseline) is arithmetically consistent with the airtime model.

However, this review found **four defects that must be fixed before the branch state is trusted**, two of which the benchmark could not have caught because the run never exercised those paths:

| # | Finding | Severity | One-line statement |
|---|---|---|---|
| P1 | `self._kf_req` is **never initialized** in `ImageRxDaemon` | **CRITICAL** | The first reassembly timeout or decode error raises `AttributeError` inside `_rx_worker` — the RX daemon's core loop dies exactly when the link degrades, which is when it matters most. The 21-frame benchmark had 0 timeouts/0 decode errors, so it never fired. |
| P2 | The "fail LOUD" PHY contract check is **silenced** at both call sites | **HIGH** | `verify_modem_matches_profile()` is wrapped in `except Exception: LOG.warning(...continuing)` in both daemons — the tripwire that motivated the whole T2 effort is demoted to a log line. |
| P3 | The `PHY_IMAGE` alias flip (BW500→BW250) **collapses every legacy 25 ms-capped consumer** | **HIGH** | 25 ms-capped chunk: 37 B → **2 B** (verified by execution). Production camera byte budget (`LIFETRAC_FRAGMENT_BUDGET=12`, profile `"image"`): 444 B → **~24 B/frame**. `tests/test_lora_proto.py` (2 failures) and `test_telemetry_fragmentation_fuzz.py` (1 error, "264 fragments > 256") now fail — verified by running them. |
| P4 | The new `test_image_tx_rx_optimization.py` **fails deterministically** | MEDIUM | Its own `sys.path` insert order re-creates the documented T3 `image_pipeline` package collision; `from image_rx_daemon import KeyframeRequester` can never import. The implementation doc's "Python Test Suite passed 100 %" is falsified (verified: `FAILED (errors=1)` in isolation). |

Plus one evidence-quality finding (§4): the benchmark's own counters (`frames_in=50, ok=20, drop_stale=30, drop_full=0` in a ~10 s window with a 4-deep queue) are **mutually inconsistent with the committed code** — no arrival pattern produces 30 stale drops with zero queue-full drops through a 4-slot queue. The most likely explanation is that the on-device file (pushed via `/tmp` bind-mount) was an intermediate iteration, not the committed one. The number 388 B/s is plausible and model-consistent; the run should nonetheless be repeated from a clean committed state with full logs archived under `bench-evidence/`.

None of this diminishes the direction: **the measured 2.15× is real headroom banked**, and the remaining fixes are small. §6 gives the fix code; §7 quantifies the speed picture and the remaining ladder.

---

## 2. Change-by-change verification

Every row personally verified against the tree (not the implementation doc's description of it).

| Claimed change | Verdict | Notes |
|---|---|---|
| `PHY_IMAGE_BW250` / `PHY_IMAGE_BW500` + alias flip in `lora_proto.py` | ✅ landed / ⚠ P3 | Alias flip done *without* migrating the 25 ms-capped consumers — see §5-P3 |
| `max_image_fragment_body()` / `pack_image_fragments()` / `pack_image_fragments_v2()` | ✅ landed, correct | Matches §9.4-A/§10.4-C of the review; `max_image_fragment_body()` = 207 B at the 170 ms default cap (verified by execution) |
| `LINK_PHY_NAMES` extended (appended, indices 0–4 preserved) | ✅ wire-compatible | X8 mirror in `camera_service.py` **not** extended — `CMD_LINK_PROFILE` index 5/6 will be rejected by the tractor (graceful, but the doc's own "MUST agree" comment is now half-true) |
| `AirtimeBudget` rolling token bucket in `image_tx_daemon.py` | ✅ landed, faithful | 380 ms/1 s rolling; conservative vs firmware fixed-anchor as designed |
| Budget-paced `_tx_one_frame` with retry + abort | ✅ landed / ⚠ P5 | Single `for attempt in range(1+4+1)` pool means a persistently NAK'd fragment can radiate up to **6×**, not the designed 1 RF retry; attempt-level failures inflate `recent_frag_loss_rate()` and can spuriously arm keyframe redundancy |
| Stale-frame cancellation (`FRAME_MAX_AGE_MS=10000`) | ✅ landed | Semantics match §10.4-D |
| Goodput stats (`goodput=… B/s`, `drop_stale`, `bytes_tx_ok`) | ✅ landed | Phase-0.1 satisfied on TX side |
| `verify_modem_matches_profile()` (regs 0x1D/0x1E/0x20/0x21) | ✅ landed / ⚠ P2 | Correct decode incl. BW bits map; **but silenced by caller try/except in both daemons** |
| `cfg_set_checked()` + `verify_active_profile()` wired into all 4 CFG calls | ✅ landed, improved | Implementer correctly used `CFG_DATA_URC` for the CFG_GET readback (my §10.4-A sketch wrongly said `CFG_OK_URC`) — credit |
| `KeyframeRequester` + poke sites (incl. silence branch) | ⚠ P1 | Class + 3 poke sites exist; **`self._kf_req` assignment does not** (verified by regex over the file: no `self._kf_req =` anywhere) |
| `FragmentReassembler.tick()` + idle-loop call | ✅ landed | Matches §10.4-D-b |
| `_encode_tile` grayscale-retry-then-`None` + `_build_frame` guards | ✅ landed, complete | Both the direct and `encode_cache` branches handle `None` — the cache-poisoning trap was avoided |
| Makefile: FHSS objects added to `check-cfg-profile(-wire)` | ✅ landed | Exactly §10.4-F |
| `check-tx-len-guard` + `tx_len_guard.c` | ✅ landed, **passes** | Re-run by this reviewer: `PASS tx_len_guard`. Hygiene: local stubs' signatures (`host_uart_send_urc` 4-arg vs real 5-arg, `sx1276_rx_arm`) don't match the real prototypes — links via C's lack of type-checking; should include the real headers |
| `test_phy_golden_vectors.py` | ✅ landed, **passes** | Both vectors (33 B→35 968 µs, 49 B→48 768 µs) pin the estimator; ±1 µs delta held |
| `test_image_tx_rx_optimization.py` | ❌ P4 | Deterministic import failure (verified); never passed as committed |
| `test_telemetry_fragmentation.py` updates | ✅ passes | Correctly migrated legacy 25 ms vectors to `PHY_IMAGE_BW500`; **the same migration was not applied to `test_lora_proto.py` / fuzz** (P3) |
| VER warm-up ×3 retry, `drain_boot` settle 1.5→2.5 s, MQTT v2 callback shims | ✅ landed / ⚠ P7 | The paho callback shims are **uncommitted working-tree edits**; the monitor harness is untracked — the hardware ran code that is not fully in git |
| Deploy-script fixes (`method_g_stage1_probe.py` pushed, adb path, `run_live_radio_monitor.ps1`) | ✅ landed / untracked | Harness hardcodes user-specific `adb.exe` and `python.exe` paths — works on this bench only |

---

## 3. What the implementation got *right* beyond the plan

Credit where due — these are better than my §9.4/§10.4 sketches:

1. **`CFG_DATA_URC` correction** in `verify_active_profile()` — the sketch had the wrong URC type; the implementer read `handle_cfg_get()` and fixed it.
2. **`encode_cache` `None` guard** — the plan's snippet only showed the direct-encode branch; the implementation also guarded `encode_cache.store(...)`, avoiding caching a `None`.
3. **Silence-branch keyframe poke** — the RX idle loop mirrors timeout counters *and* pokes the requester there, which is precisely when it matters (would matter — see P1).
4. **`tx_len_guard` link recipe** pulls the real `sx1276_tx.c` + `sx1276_airtime.c` + FHSS objects rather than stubbing the unit under test — the guard is pinned against the production code path, `-DLIFETRAC_FHSS_TX_ROUTED` included.
5. **Golden-vector test survived the alias flip** because it names `image_bw250` explicitly — exactly the drift-resistance the T2 design intended.

---

## 4. What the hardware benchmark proves — and what it doesn't

### 4.1 The 388.0 B/s number is model-consistent

A 194 B TileDeltaFrame → one fragment (4 B header + 194 B ≤ 207 B chunk) → 206 B on-air → **ToA ≈ 164 ms** at SF7/BW250/CR4-5. Both the firmware's fixed-anchor window (⌊400/164⌋ = 2/window) and the host's rolling 380 ms budget (2×164 = 328 ≤ 380 < 3×164) admit **exactly 2 fragments/s** → 2 × 194 = **388 B/s**. The measured stats line (`3880 B over the ~10 s stats window`) matches to the byte. This is a genuine validation of the airtime model and the pacing implementation.

### 4.2 Three qualifications the implementation doc omits

1. **Offered load equalled the ceiling.** The generator publishes exactly 2 fps × 194 B. The benchmark therefore *cannot distinguish* "link saturated at 388 B/s" from "generator only offered 388 B/s" — both produce the same number. A saturation test must offer *more* than capacity (see §7.3).
2. **Only the single-fragment path was exercised.** 194 B < 203 B chunk ⇒ every frame was 1 fragment. Multi-fragment reassembly, the retry loop, frame abort, v2 redundancy, and keyframe latency were **never touched by RF**. The doc's "keyframe ~6.5 s" and "P-frame ~170 ms" rows are extrapolations from the model (fine — but they are labelled "Live Hardware Benchmark").
3. **The counters are internally inconsistent with the committed code.** `frames_in=50, ok=20, drop_stale=30, drop_full=0` cannot coexist with a 4-deep `queue.Queue`: 30 frames can only age past 10 s if ≥20 further frames arrived while they queued, which forces `drop_full > 0` through a 4-slot queue under every arrival pattern. Most plausible: the board ran an intermediate `/tmp`-pushed iteration (different queue depth or stale logic), not commit `476e41d2`. Additionally `rx_frames=21` vs TX `ok=20` (off-by-one — likely a leftover warm-up frame). **Recommendation:** re-run from the committed state, archive full logs + counter JSON under `bench-evidence/` per HIL conventions, and let the numbers be reproducible. This is the same evidence-hygiene standard the project already learned to demand (see misdiagnosis notes).

### 4.3 Scorecard

| Plan item | Bench-proven? |
|---|---|
| Closed-loop pacing sustains QoS-ceiling rate, zero `ABORT_QOS` | ✅ yes (2/s sustained, no FORBIDDEN in logs) |
| PHY contract check runs on live modem | ✅ yes (both daemons logged `PHY contract OK`) |
| RSSI/SNR healthy at bench range | ✅ (−41 dBm / +9.5 dB) |
| Multi-fragment frames, retry, abort, v2 copies | ❌ not exercised |
| Keyframe latency, `req_keyframe` self-heal | ❌ not exercised (and P1 would crash it) |
| Camera-service budget path (`LIFETRAC_USE_LORA_BRIDGE=1`) | ❌ bypassed by host-side generator (P3 hides here) |

---

## 5. New findings (full detail)

### P1 — `ImageRxDaemon._kf_req` never initialized — CRITICAL

`KeyframeRequester` is defined and `self._kf_req.poke(...)` is called at three sites (idle branch + two counter-mirror sites), but **no `self._kf_req = ...` exists anywhere in the file** (verified). First reassembly timeout → `AttributeError` → depending on branch, the exception either kills `_rx_worker` (idle branch is *outside* any try) or is caught as a `reassembler.feed crashed` — either way the self-heal feature is dead and the daemon can zombie. The benchmark never hit it because nothing timed out. Fix in §6.1.

### P2 — Fail-loud demoted to fail-whisper — HIGH

Both daemons wrap the contract check in the same `try/except` as profile configuration:

```python
try:
    configure_regulatory_profile_if_needed(link)
    verify_modem_matches_profile(link, PHY_IMAGE_BW250)
except Exception as exc:
    LOG.warning("regulatory profile config / contract check failed: %s", exc)   # ← continues!
```

A BW-mismatched modem now produces one WARNING line and a daemon that happily missizes every fragment — precisely the F1 failure mode T2 was built to make impossible. The check must abort startup (that is its entire value); profile-config failures can stay soft. Fix in §6.2.

### P3 — Alias flip poisoned the legacy 25 ms consumers — HIGH

Changing `PHY_IMAGE` to BW250 while `TELEMETRY_FRAGMENT_MAX_AIRTIME_MS = 25.0` stayed global reproduces the F13 pattern in mirror image. Verified by execution:

- `max_telemetry_fragment_payload(PHY_IMAGE)`: **37 B → 2 B** per fragment.
- X8 `image_pipeline/fragment.py` (`pack_image_fragments`, `estimate_fragment_count`, `max_payload_for_n_fragments`) all default to `PHY_IMAGE` @ 25 ms → chunk = 2 B.
- `camera_service._compute_link_bytes(12, "image")` → **~24 B/frame** byte budget in the production compose (was 444 B) — the bridge-mode camera will ship ~1 minimal tile per frame and the UI will crawl, *slower than before the optimization*.
- `tests/test_lora_proto.py`: **2 failures** (`35.968 not ≤ 25.0`); `test_telemetry_fragmentation_fuzz.py`: **1 error** (264 fragments > 256). Both verified by running them. These correspond to the "3 fail" recorded in the interim CI-discovery snapshot — they are *this* regression, not pre-existing noise.

The strict-path daemon avoids all of this (it uses the new 170 ms sizer), which is why the bench run looked clean. Fix in §6.3: migrate the X8 fragment helpers and the camera budget to the image sizer; re-pin the legacy 25 ms tests on `PHY_IMAGE_BW500` (as `test_telemetry_fragmentation.py` already did) or on the new cap.

### P4 — The new test module cannot import — MEDIUM

`test_image_tx_rx_optimization.py` does `sys.path.insert(0, p)` in the order `(_BASE_STATION, _FIRMWARE_X8, _X8_HELPER)` → final precedence `_X8_HELPER > _FIRMWARE_X8 > _BASE_STATION` → `image_pipeline` resolves to the **X8-side** package (which has no `frame_format`/`reassemble`) → `import image_rx_daemon` explodes. This is the exact T3 collision documented in `run_tests.ps1`, reintroduced inside the optimization's own test. Verified: fails in isolation too. Fix in §6.4.

### P5 — Retry loop can radiate one fragment up to 6× — LOW-MED

The single attempt pool (`range(1 + 4 + 1)`) doesn't separate QoS refusals (no RF) from RF failures: a fragment whose `TX_DONE` repeatedly returns non-OK re-radiates up to 6 times (each admitted and budget-recorded, so paced — but the design intent was ≤1 RF retry). Side effect: `fragments_tx_fail` counts *attempts*, so `recent_frag_loss_rate()` overestimates PER and can arm `copies=2` keyframe redundancy spuriously (doubling keyframe airtime on a healthy link). Fix in §6.5.

### P6 — `req_keyframe` topology gap — MEDIUM (design, pre-existing but now relevant)

`KeyframeRequester` publishes to the **base-side** broker. In the bench harness both daemons share the Windows broker, so it would work (post-P1). In the production compose topology (tractor-local mosquitto vs base broker) the message never reaches `camera_service` — there is no bridge for `lifetrac/v25/cmd/req_keyframe`. Options: bridge that one topic (mosquitto `connection`/`topic` stanza), or carry `CMD_REQ_KEYFRAME` over the air via the base L072 (the RX daemon owns the UART and *can* send `TX_FRAME_REQ`; the tractor TX daemon would need a small RX path). Document the limitation until then.

### P7 — Provenance: benchmark ran non-committed code — MEDIUM (process)

Uncommitted daemon edits (paho v2 callback shims) + untracked harness + `/tmp` bind-mount deployment = the flashed bench state is not reconstructible from git. This is exactly the "works only because containers are hot-patched" risk the v25.0.2 roadmap milestone exists to kill. Commit the shims and the harness; re-run; archive evidence.

### P8 — Hygiene (no action blocked on these)

- `tx_len_guard.c` stub prototypes diverge from the real headers (`host_uart_send_urc` 4-arg vs 5-arg; `sx1276_rx_arm(uint8_t)` vs `bool sx1276_rx_arm(void)`) — links only because C doesn't type-check across TUs; include the real headers and match signatures.
- Harness hardcodes `C:\Users\dorkm\...` paths for adb/python; parameterize.
- TX container uses `arduino-ootb-python-devel:738bc44`, RX uses `lifetrac-v25:latest` — pin both by digest (roadmap v25.0.2).
- `camera_service.LINK_PHY_NAMES` mirror not extended with the two new names (indices 5/6 rejected by tractor — degrade-safe but asymmetric).

---

## 6. Fix list with code (priority order)

### 6.1 P1 — initialize the requester (`image_rx_daemon.py`, one line + import note)

```python
# In ImageRxDaemon.__init__, after self._client = None:
        self._client = None
        # P1 fix: _kf_req must exist before _rx_worker ever runs. The
        # client_supplier closure returns the *current* client so pokes
        # before MQTT connect are safely dropped by KeyframeRequester.
        self._kf_req = KeyframeRequester(lambda: self._client)
```

And pin it with a test that would have caught this (goes in the repaired `test_image_tx_rx_optimization.py`):

```python
    def test_rx_daemon_has_kf_req(self):
        d = ImageRxDaemon(uart="loop://", baud="0", mqtt_host="127.0.0.1",
                          mqtt_port=1883, reassembler_timeout_ms=1500)
        d._kf_req.poke("attribute-exists smoke")   # must not raise
```

### 6.2 P2 — let the tripwire trip (both daemons, `_open_link`)

```python
        try:
            configure_regulatory_profile_if_needed(link)
        except Exception as exc:                              # profile setup stays soft
            LOG.warning("regulatory profile config failed: %s", exc)
        # T2 contract check is fail-CLOSED by design: a mismatched modem
        # means every fragment is mis-sized (F1). Do not soften.
        verify_modem_matches_profile(link, PHY_IMAGE_BW250)   # raises RuntimeError
```

Optional escape hatch for bench experiments, default off: `if os.environ.get("LIFETRAC_SKIP_PHY_CONTRACT") != "1": verify_modem_matches_profile(...)`.

### 6.3 P3 — migrate the legacy consumers to the image sizer

```python
# --- firmware/tractor_x8/image_pipeline/fragment.py ----------------------
from lora_proto import (
    PHY_IMAGE_BW250, IMAGE_FRAG_AIR_CAP_MS,
    max_image_fragment_body, pack_image_fragments as _pack_image_fragments,
    TELEMETRY_FRAGMENT_HEADER_LEN, TELEMETRY_FRAGMENT_MAGIC,
)

IMAGE_FRAGMENT_MAGIC = TELEMETRY_FRAGMENT_MAGIC
IMAGE_FRAGMENT_HEADER_LEN = TELEMETRY_FRAGMENT_HEADER_LEN
IMAGE_FRAGMENT_MAX_AIRTIME_MS = IMAGE_FRAG_AIR_CAP_MS        # was 25.0

def pack_image_fragments(payload, frag_seq, *, profile=PHY_IMAGE_BW250,
                         max_air_ms=IMAGE_FRAG_AIR_CAP_MS):
    return _pack_image_fragments(payload, frag_seq, profile, max_air_ms)

def _chunk(profile, max_air_ms):
    return max_image_fragment_body(profile, max_air_ms) - IMAGE_FRAGMENT_HEADER_LEN

def estimate_fragment_count(payload_len, *, profile=PHY_IMAGE_BW250,
                            max_air_ms=IMAGE_FRAG_AIR_CAP_MS):
    c = _chunk(profile, max_air_ms)
    if c <= 0:
        raise ValueError(f"profile {profile.name} cannot fit any fragment in {max_air_ms} ms")
    return max(1, (payload_len + c - 1) // c)

def max_payload_for_n_fragments(n_fragments, *, profile=PHY_IMAGE_BW250,
                                max_air_ms=IMAGE_FRAG_AIR_CAP_MS):
    return max(0, n_fragments * _chunk(profile, max_air_ms))
```

With that, `camera_service._compute_link_bytes(12, "image")` returns 12 × 203 = **2436 B** (vs 24 B broken / 444 B pre-optimization) — the bridge-mode camera budget finally reflects the real link. Then re-pin the two failing legacy tests the same way `test_telemetry_fragmentation.py` already was:

```python
# tests/test_lora_proto.py — the two 25 ms assertions test the LEGACY cap,
# which only ever held at BW500 (F1 postmortem). Pin them explicitly:
        self.assertLessEqual(lora_time_on_air_ms(32, PHY_IMAGE_BW500), 25.0)
# tests/test_telemetry_fragmentation_fuzz.py — FAST_PROFILES rides the same fix:
FAST_PROFILES = (PHY_IMAGE_BW500, PHY_CONTROL_SF7)
```

### 6.4 P4 — fix the new test's import order

```python
# tests/test_image_tx_rx_optimization.py — base_station must WIN the
# image_pipeline name (T3 collision; see run_tests.ps1 header):
for p in (_X8_HELPER, _FIRMWARE_X8, _BASE_STATION):   # reversed: last insert wins
    if p not in sys.path:
        sys.path.insert(0, p)
```

(Or, better: import `AirtimeBudget` and `KeyframeRequester` without touching `_FIRMWARE_X8` at all — nothing in this test needs the X8 package.)

### 6.5 P5 — split the retry budgets

```python
            qos_left, rf_left = 4, 1
            while True:
                if not self.budget.admit(est_us, self._stop):
                    return
                try:
                    link.send(HOST_TYPE_TX_FRAME_REQ, tx_frame)
                    done, _faults = wait_for_tx_done(link, tx_id,
                                                     timeout=PER_FRAGMENT_TX_TIMEOUT_S)
                except RuntimeError:                 # QoS refusal: no RF spent
                    qos_left -= 1
                    if qos_left < 0:
                        break
                    time.sleep(est_us / 2e6)
                    continue
                except (TimeoutError, Exception):    # RF (or unknown) spent
                    self.budget.record(est_us)
                    rf_left -= 1
                    if rf_left < 0:
                        break
                    continue
                self.budget.record(done.get("time_on_air_us") or est_us)
                if done["status"] == 0:
                    sent = True
                if done["status"] != 0:
                    rf_left -= 1
                    if rf_left >= 0:
                        continue
                break
```

And make the PER estimate fragment-level, not attempt-level: increment a separate `fragments_lost` only when a fragment is finally abandoned.

### 6.6 Re-run protocol (closes §4.2/P7)

1. Commit the working-tree shims + `run_live_radio_monitor.ps1`; tag the commit.
2. Re-run the 30 s benchmark **at ≥2× offered load** (e.g. generator `time.sleep(0.2)` → 5 fps) so the link is provably saturated; expected: goodput pins at the pacing ceiling (~388–406 B/s at this frame size), `drop_stale`/`drop_full` absorb the excess *consistently with the queue depth*.
3. Add a multi-fragment leg: one ≥1 KB synthetic keyframe per 10 s (5–6 fragments) to exercise reassembly, retry, and (with `LIFETRAC_KEYFRAME_COPIES=2`) the v2 dedup path on air.
4. Archive stdout of both daemons + the stats JSON under `bench-evidence/` with the git SHA in the folder name.

---

## 7. Speed improvements

### 7.1 Achieved (verified against model + logs)

| Metric | Before | After | Factor | Confidence |
|---|---:|---:|---:|---|
| Effective goodput (single channel, bench) | ~180 B/s | **388 B/s** | **2.15×** | High — matches model to the byte; re-run per §6.6 for provenance |
| Radio duty during transfer | ~24 % | ~33 % (2 × 164 ms/s) | 1.4× | Model-derived from logged cadence |
| P-frame (1 fragment) service time | ~400 ms | ~170 ms | 2.35× | Model-consistent; not directly timestamp-paired in logs |
| Keyframe (3 KB) | ~15–17 s | ~6.5 s (extrapolated) | ~2.4× | **Unverified** — multi-fragment path not exercised; verify in §6.6 step 3 |

### 7.2 Immediately available headroom (host-only, after §6 fixes)

1. **Fill the fragment (`+~5 %`).** The bench frames were 194 B against a 203 B chunk. Real camera frames fragmented at 203 B/fragment reach 2 × 203 = **406 B/s** at the same 170 ms cap.
2. **247 B bodies (`+~20 %`).** Raising `LIFETRAC_FRAG_AIR_CAP_MS` to 200 ms admits the full 247 B body (ToA 199.8 ms; 2 × 199.8 = 399.6 ms — fits the firmware's 400 ms window by 384 µs). Ceiling: 2 × 243 B data = **486 B/s**. Requires the margin analysis to hold on-air: the firmware books the *predicted* ToA, and 384 µs of slack is thin — validate with one `RFCO_PERTX` sweep; if aborts appear, back off to 240 B bodies (2 × 195.5 ms = 391 ms, 472 B/s).
3. **Fix P3** so the *camera* actually produces frames big enough to use the above (24 B budget ships ~1 tile; 2436 B budget ships the real frame).

### 7.3 Measurement upgrade required before further tuning

The current benchmark measures *offered load*, not capacity (§4.2-1). Add a saturation mode to the harness: offer ≥2× the expected ceiling and report the plateau. Only a plateau is a throughput measurement; everything else is a traffic report.

### 7.4 The standing ladder (unchanged from the review, §10.5 numbers)

| Step | Gate | Expected goodput |
|---|---|---:|
| Today (post-fixes, single channel) | §6 fixes | ~406–486 B/s |
| FHSS-50 closure (roadmap v25.0.6.5) + depth-2 TX mailbox | firmware | **~1.2 KB/s** |
| DTS BW500 (re-scoped per §10.1-C4: routing gate + profile-aware dwell + modem reprogram) | firmware | **~2.4 KB/s** |
| Codec: container-strip → mosaic-WebP (F8) | host | ×1.3–1.6 on top |
| SF6/implicit-header experiment (Phase 4) | bench-only | ×1.37 candidate |

The FHSS step remains the single biggest unlock (~2.5×) and is now *actually testable* because `cfg_set_checked`/`verify_active_profile` finally make profile-activation failures visible (the F15 fix landed and is wired — verified).

### 7.5 Latency (not just throughput)

With pacing in place, the biggest *perceived*-speed lever is no longer B/s: it is **P1/P6 keyframe recovery** (a lost keyframe currently stalls the canvas for up to `KEYFRAME_PERIOD_S`=60 s) and **keyframe period restoration** (60 s → 15–20 s once §7.2 lands). A 486 B/s link with 5 s keyframe recovery *feels* faster than a 1 KB/s link that stares at a stale canvas for a minute.

---

## 8. Verdict

The implementation is a genuine, hardware-validated step: the pacing/sizing/contract architecture from the review is in the tree, the C and golden-vector gates pass, and the 2.15× is banked. But the branch is **not merge-clean**: one crash bug on the exact resilience path the changes were meant to add (P1), one silenced safety check (P2), one production-budget regression with two failing pre-existing test modules (P3), one dead-on-arrival new test (P4), and a benchmark whose counters don't reconcile with the committed code (P7/§4.2). All five fixes in §6 are small, none requires firmware work, and after them the next real speed step is the FHSS closure — same as it was, but now with the instrumentation to prove it.
