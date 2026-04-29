# LifeTrac v25 Cybersecurity Case (IEC 62443 SL-1 sketch)

**Status:** DRAFT FOR REVIEW. Targets IEC 62443-3-3 **Security Level 1**
(protection against casual or coincidental violation). SL-2+ would require
a per-device PKI and rolling key infrastructure that v25 explicitly
defers — see [KEY_ROTATION.md](KEY_ROTATION.md).

**Companions:** [SAFETY_CASE.md](SAFETY_CASE.md), [KEY_ROTATION.md](KEY_ROTATION.md),
[LORA_PROTOCOL.md](LORA_PROTOCOL.md), [DECISIONS.md](DECISIONS.md).

## 1. System under consideration

A single-tenant LifeTrac v25 installation: one base station, one tractor,
1–2 handhelds, optional autonomy node, deployed on a private 2.4 GHz WiFi
SSID **plus** a 915 MHz LoRa link. Off-the-grid by default; no inbound
internet exposure.

## 2. Trust zones (IEC 62443-3-2)

| Zone | Members | Trust posture |
|------|---------|---------------|
| Z-LORA | handheld, tractor M7, base lora_bridge | Mutually authenticated by AES-256-GCM with shared fleet key |
| Z-LAN  | base X8 (uvicorn :8080), operator browser, tractor X8 (MQTT) | Behind dedicated SSID; mutual auth via PIN cookie |
| Z-INET | none in v25 | n/a |

Z-LORA ↔ Z-LAN crossing point is `base_station/lora_bridge.py` — it
unwraps GCM, validates source priority, then publishes plaintext on the
local MQTT broker. Z-LAN consumers (web_ui, audit_tail, image pipeline)
implicitly trust anything on the broker; the broker is bound to
127.0.0.1 inside the docker network for exactly this reason
(see [docker-compose.yml](docker-compose.yml)).

## 3. Foundational requirements (IEC 62443-3-3 FR-1..7) coverage

| FR | Requirement (paraphrase) | Met by |
|----|---------------------------|--------|
| FR-1 IAC | Identification & access control | LIFETRAC_PIN cookie (web_ui.py); fleet key for radio (KEY_ROTATION.md) |
| FR-2 UC  | Use control | Source-priority arbitration in tractor_m7.ino; UI controls disabled when handheld active |
| FR-3 SI  | System integrity | AES-256-GCM tags; replay window; audit log per audit_log.py |
| FR-4 DC  | Data confidentiality | GCM encryption end-to-end on radio; HTTPS deferred to a future deployment because v25 LAN is air-gapped |
| FR-5 RDF | Restricted data flow | mosquitto bound to docker-internal network; only :8080 exposed; reverse-proxy adds TLS at site boundary |
| FR-6 TRE | Timely response to events | audit_log.py JSONL appends inline; operator UI surfaces lockouts |
| FR-7 RA  | Resource availability | Three-watchdog de-energise on radio loss (see SAFETY_CASE.md §4) |

## 4. Threat model (STRIDE per zone crossing)

### 4.1 Operator → web_ui (Z-LAN)

| STRIDE | Threat | Mitigation |
|--------|--------|------------|
| S | Operator impersonates another | Single-tenant; PIN-only; out of scope for SL-1 |
| T | Replay an `/api/estop` POST | Idempotent — replay just re-sends E-STOP |
| R | Operator denies pressing E-STOP | audit_log records every authenticated POST |
| I | Sniff PIN over LAN | LAN is dedicated SSID; reverse-proxy adds TLS for site-edge deployments |
| D | Brute-force PIN | 5-fail lockout, 60 s cool-off (web_ui.py) |
| E | Privilege escalation | All control endpoints share the same PIN; no per-route role split in SL-1 |

### 4.2 RF attacker → tractor M7 (Z-LORA)

| STRIDE | Threat | Mitigation |
|--------|--------|------------|
| S | Forge ControlFrame from "handheld" | Unknown fleet key → GCM tag rejected |
| T | Replay captured ControlFrame | Replay window in lp_decrypt + nonce monotonicity |
| R | n/a | n/a |
| I | Decrypt traffic | AES-256-GCM with 256-bit shared key; no known break |
| D | Jam channel | Detected by the H7 missed-frame timeout → safe state |
| E | n/a | n/a |

### 4.3 RF attacker → operator (false telemetry)

| Threat | Mitigation |
|--------|------------|
| Forge "telemetry healthy" to hide a stuck valve | Telemetry is also AES-GCM-protected; an attacker without the key cannot mint a frame the operator's UI will accept |

## 5. Key management (SL-1 scope)

- **Single fleet key**, generated on the install bench by `tools/keygen.py`
  and provisioned to every node by `tools/provision.py --write-port`.
- Rotation cadence: **annual** OR **on operator suspicion of compromise**,
  whichever comes first. See [KEY_ROTATION.md](KEY_ROTATION.md).
- Old key remains valid for a 24-h overlap window so out-of-service nodes
  can re-pair without coordination.
- No HSM in v25; key lives in firmware flash. Acceptable at SL-1 because
  physical access to a tractor implies game-over anyway.

## 6. Audit & incident response

- `audit_log.py` writes one JSONL line per auth attempt, source change,
  GCM rejection, replay rejection, and SF-ladder transition.
- Rotated at 10 MB × 5 keep, ~50 MB ceiling.
- No automatic SIEM forwarding in v25 — the operator is expected to
  inspect locally via `/audit` after any incident.

## 7. Known SL-1 → SL-2 gap

To raise to SL-2 the design would need:

1. Per-device PKI (replaces single fleet key).
2. TLS on the LAN (web_ui + MQTT).
3. Role-based access on web_ui (operator vs. service tech).
4. Tamper detection on the tractor enclosure.
5. Signed audit log with rotating tamper seal.

These are explicitly out of scope for v25 and are tracked in TODO.md.

## 8. Verification matrix

| Control | Test | Where defined |
|---------|------|---------------|
| GCM rejection | `firmware/bench/crypto_vectors/host_check.c` against base_station/tests/test_crypto_vectors.py | base_station/tests/ |
| Replay window | `base_station/tests/test_replay_window.py` | base_station/tests/ |
| PIN lockout | `base_station/tests/test_web_ui_auth.py` | base_station/tests/ |
| Audit append | `base_station/tests/test_audit_log.py` | base_station/tests/ |

## 9. Build-config attack surface (BC-13)

The build-config subsystem (TOML files, installer bundles, `lifetrac-config`
CLI, push daemon, `audit.jsonl` `config_loaded` events, generated firmware
header) is itself an asset class with safety significance: a maliciously crafted
config can change physical-shape capability flags (e.g. enable `has_lift_arms`
on a unit with no lift cylinders, or raise a hydraulic limit beyond what the
plumbing tolerates) without ever touching firmware code. This section pins the
SL-1 threat model for that surface; each row references concrete artefacts that
the SIL gate ([test_cybersecurity_buildconfig_xref_sil.py](base_station/tests/test_cybersecurity_buildconfig_xref_sil.py))
verifies still exist.

### 9.1 Build-config zone (Z-CONFIG)

| Asset | Lives on | Crosses into |
|-------|----------|--------------|
| `build.<unit_id>.toml` source | install-bench laptop (Z-LAN-bench) | Z-CONFIG when bundled |
| `lifetrac-config-<unit>-<sha8>.toml` bundle | USB stick / LAN | Z-CONFIG on the tractor X8 |
| Active config under `/etc/lifetrac/` | tractor X8 (Z-LAN-tractor) | Z-CONFIG (read by web_ui, lora_bridge) |
| `config_loaded` audit events | `audit.jsonl` on tractor X8 | Z-AUDIT (operator inspection) |
| Generated firmware header | repo + flashed M7 | Z-FIRMWARE (codegen `--check` is the drift gate) |

### 9.2 STRIDE per build-config asset

| Asset / action | STRIDE | Threat | Mitigation |
|---|---|---|---|
| `build.toml` on bench | T | Edit a TOML to enable a capability the hardware lacks | `lifetrac-config validate` schema gate; BC-09 capability cross-reference forces a BOM row to exist for every physical-shape leaf |
| Bundle on USB stick | T | Swap bundle contents in transit | `lifetrac-config verify` re-hashes; filename embeds `sha8` and `unit_id` so a swap to another unit's bundle is visible |
| Bundle on USB stick | S | Hand-craft a bundle whose embedded `unit_id` does not match the target tractor | `verify` rejects `unit_id` mismatch; activation refuses if filename `unit_id` ≠ embedded `unit_id` |
| `lifetrac-config push` over LAN | S | RF-attacker-as-installer pushes a bundle | Z-LAN PIN-cookie auth on the install daemon endpoint (same control as web_ui) |
| `lifetrac-config push` over LAN | E | Operator on the LAN escalates to bypass schema validation | Daemon re-runs `validate` server-side; `--force` is rejected at SL-1 |
| Active config on tractor | T | Edit `/etc/lifetrac/build.toml` in place after install | Atomic-replace install path means an in-place edit changes the on-disk `config_sha256`; next `config_loaded` event records the drift; BC-14 inventory surfaces it as a new row |
| `config_loaded` audit event | R | Operator denies which build was active during an incident | Every web_ui and lora_bridge process start emits one `config_loaded` JSONL line with `unit_id` + full `config_sha256`; BC-14 `lifetrac-config inventory` aggregates across the fleet |
| `config_loaded` audit event | T | Tamper with `audit.jsonl` to hide a swap | Audit log is append-only at SL-1; signed/sealed log is the SL-2 gap (see —7 item 5) |
| Generated firmware header | T | Ship a header that diverges from the active TOML | `lifetrac-config codegen --check` is a CI drift gate; BC-09 binds every physical leaf to a BOM row so silent capability inflation fails the bidirectional schema parity check |

### 9.3 Capability-altering leaves are safety-significant

Physical-shape leaves enumerated in [HARDWARE_BOM.md](HARDWARE_BOM.md) —
`Capability cross-reference` (Round 36 / BC-09) are the subset of build-config
leaves whose tampering can drive an actuator the unit cannot safely command.
For SL-1 the mitigation is **defence in depth**, not signed configs:

1. Schema validation rejects out-of-range values (`lifetrac-config validate`).
2. BOM cross-reference forces a hardware row to exist for every physical leaf
   (BC-09 bidirectional gate).
3. `codegen --check` rejects any divergence between the active TOML and the
   compiled firmware header.
4. `config_loaded` audit + BC-14 inventory mean a swap is *visible* to the
   operator within one boot.
5. The three-watchdog de-energise (SAFETY_CASE.md —4) is the last-resort
   safe-state regardless of what the config claims is plumbed.

The SL-2 gap that closes the residual risk (signed bundles + per-device PKI)
is tracked in —7 item 1.

### 9.4 Drift gate

The SIL gate
[test_cybersecurity_buildconfig_xref_sil.py](base_station/tests/test_cybersecurity_buildconfig_xref_sil.py)
pins this section against the rest of the codebase: every `lifetrac-config`
subcommand named here exists in the CLI; every relative link resolves on disk;
the STRIDE table covers each Z-CONFIG asset exactly once; and this section
self-references its enforcing test file (so renaming the test fails the gate).
