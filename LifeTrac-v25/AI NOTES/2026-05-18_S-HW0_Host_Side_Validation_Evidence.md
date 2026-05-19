# S-HW.0 host-side validation evidence — 2026-05-18

Host-side validation of D13 (AES-GCM-64 implicit-nonce codec), D14 (image
plaintext+CRC32 framer), and the host-boundary class-tag enforcer, executed
on a real Portenta X8 (ARM64, Foundries.io LmP). No RF; UDP localhost as a
transport-shaped stand-in for the LoRa link.

Why no RF: L072 LoRa radios are detached per S-HW.1, and even if attached
the shipping L072 firmware does not yet speak the D13/D14 wire format.
S-HW.0 closes the host-side correctness gap so that when S-HW.1 lands the
only remaining unknown is RF behavior, not codec/replay/enforcer logic.

## Devices

| Role | ADB serial | Python | cryptography |
|------|------------|--------|--------------|
| X8-A (full Python rootfs) | 2D0A1209DABC240B | 3.10.9 | 36.0.2 |
| X8-B (camera, stripped Python rootfs) | 2E2C1209DABC240B | 3.10.4 | n/a (no `dataclasses`/`socket` in stripped stdlib) |

X8-B was excluded from host-side validation: per
`/memories/repo/lifetrac-x8-python-stripped.md`, the Foundries.io LmP rootfs
omits `dataclasses`, `socket`, `json`, `logging`. Production code on X8-B
runs inside the Docker container, which is out of scope for this bench
(would require building/pushing a container, far beyond the value of
validating that the same `cryptography` library produces the same wire bytes
on the second machine). X8-A is sufficient because the bench validates
codec correctness, not network heterogeneity.

## Check B — AES-GCM-64 throughput microbench (`bench_crypto_perf.py`)

Per-operation cost (mean of 2000 iters on X8-A):

| Payload | Profile | enc (ms) | dec (ms) | round-trip (ms) | CPU % @ design cadence |
|---|---|---|---|---|---|
| 16 B ControlFrame, 20 Hz | GCM-128 explicit | 0.314 | 0.273 | 0.587 | 1.175 % |
| 16 B ControlFrame, 20 Hz | **GCM-64 implicit (D13)** | 0.424 | 0.446 | 0.870 | **1.740 %** |
| 32 B image frag, 10 Hz | GCM-128 explicit | 0.307 | 0.271 | 0.578 | 0.578 % |
| 32 B image frag, 10 Hz | **GCM-64 implicit (D13)** | 0.415 | 0.445 | 0.859 | **0.859 %** |
| 100 B telemetry, 5 Hz | GCM-128 explicit | 0.316 | 0.274 | 0.590 | 0.295 % |
| 100 B telemetry, 5 Hz | **GCM-64 implicit (D13)** | 0.416 | 0.454 | 0.869 | **0.435 %** |

D14 image plaintext+CRC32 (no crypto): pack 13.3 µs, unpack 15.8 µs.

### Findings

1. **Aggregate cost of full D13 cutover at the design cadence is
   1.740 + 0.859 + 0.435 ≈ 3.03 % of one A53 core.** This is well under
   any reasonable budget; the gst encode pipeline dominates CPU on this
   SoC by 2+ orders of magnitude. No risk to A/V from the D13 cutover.

2. **D13 is ~50 % slower per op than the shipped D-default GCM-128
   explicit path.** Root cause is API choice: D13 uses the low-level
   `Cipher(AES(key), GCM(nonce, tag, min_tag_length=8))` for tag
   truncation, which has more per-call Python overhead than the
   dedicated `AESGCM(key)` class used by the shipped path. This is
   acceptable — the absolute cost (0.87 ms) is still negligible — but
   it means **D13 is NOT cheaper than D-default on CPU**; D13 is only
   cheaper on AIR-TIME. The §17 motivation for D13 is unchanged
   (saving 16 B of overhead = ~33 % ToA reduction on control frames),
   but the secondary claim "less CPU work" if it appears anywhere in
   the design notes is false and should be struck.

3. **D14 path costs ~30 µs per fragment** (pack+unpack), trivially
   cheap.

## Check C — UDP loopback integration harness (`bench_loopback_d13_d14.py`)

All 12 assertions PASS on X8-A:

```
== D13 AES-GCM-64 implicit-nonce round-trip over UDP ==
  [PASS] control wire size
  [PASS] control round-trip
== D14 image plaintext+CRC32 round-trip over UDP ==
  [PASS] image wire size
  [PASS] image round-trip
== D13 MAC-tampered frame rejected by AEAD ==
  [PASS] tampered tag rejected
== D13 boot_ctr mismatch rejects everything ==
  [PASS] boot_ctr+99 rejects all 5 frames
== ReplayWindow rejects duplicate D13 seq ==
  [PASS] first arrival accepted
  [PASS] replay rejected by ReplayWindow
== Host-boundary class-tag enforcer over UDP ==
  [PASS] P3 + D14 plain accepted
  [PASS] P0 + D14 plain REJECTED (split-trust guard)
  [PASS] P0 + GCM-64 implicit accepted
== Sustained 20 Hz control loop over UDP for 2 s ==
  [PASS] ≥ 35 frames decoded+accepted in 2 s
  [PASS] 0 MAC/replay losses on lossless localhost
  (info: 35 accepted, 0 rejected over 2 s @ 20 Hz)
```

### What this proves

- The new D13 codec and D14 framer **produce wire bytes that survive a
  real socket boundary on the production ARM64 Python runtime** with
  the exact `cryptography` 36.0.2 library version shipped on the LmP
  rootfs. Codec correctness is not a dev-machine artifact.
- The four split-trust guarantees from §20 hold end-to-end:
  - tag forgery rejected by AEAD,
  - boot_ctr replay across reboots rejected,
  - within-boot replay rejected by `ReplayWindow`,
  - **a perfectly-valid D14 fragment cannot be laundered into the P0
    pipeline** — the class-tag enforcer raises `ClassTagViolation`
    even though `unpack_image_fragment_plain` happily decoded it.
  This is the key safety property: an attacker who can spoof
  unauthenticated image fragments (the only legitimate D14 use)
  cannot escalate them into ControlFrame/E-STOP commands at the host
  boundary.
- The shipped `cryptography` library does support GCM tag truncation
  to 64 bits via the low-level `Cipher(...)` API. This was an open
  question that could only be answered on the target.

### What this does NOT prove

- Anything about RF behavior (S-HW.1 still gates that).
- Anything about latency tail under packet loss / out-of-order
  delivery (UDP localhost is essentially lossless).
- Behavior on X8-B inside Docker (would need a container build).
- Integration with the live `lora_bridge.py` TX path
  (still S5.1b — the codec is exercised by the bench but not yet by
  the production bridge).

## Files added (this evidence pass)

- `LifeTrac-v25/DESIGN-CONTROLLER/base_station/bench_crypto_perf.py`
- `LifeTrac-v25/DESIGN-CONTROLLER/base_station/bench_loopback_d13_d14.py`
- this evidence doc

## Reproduce

```powershell
cd LifeTrac-v25/DESIGN-CONTROLLER/base_station
adb -s 2D0A1209DABC240B push bench_crypto_perf.py     /tmp/
adb -s 2D0A1209DABC240B push bench_loopback_d13_d14.py /tmp/
adb -s 2D0A1209DABC240B push lora_proto.py             /tmp/
adb -s 2D0A1209DABC240B shell "cd /tmp && python3 bench_crypto_perf.py"
adb -s 2D0A1209DABC240B shell "cd /tmp && python3 bench_loopback_d13_d14.py"
```

## Verdict

S-HW.0 PASS. D13 + D14 + class-tag enforcer are cleared for production
cutover on the host side. Remaining cutover work (S5.1b) is pure
integration — wiring `lora_bridge.py` from the shipped `encrypt_frame`
call site to `encrypt_frame_gcm64_implicit`, plus persisting `boot_ctr`
to flash, plus the link-up handshake frame — and is no longer gated on
any unknowns about the codec, the library, or the ARM64 runtime.
