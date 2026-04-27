# LifeTrac v25 Controller Implementation Review - Copilot v1.1

Date: 2026-04-27
Scope: Review of the recent master-plan implementation slice in `DESIGN-CONTROLLER/`.

## Executive finding

The active implementation now follows the master-plan structure much more closely, but the implementation surfaced one blocking design mismatch: **SF7/BW125 cannot carry the current encrypted 20 Hz `ControlFrame` stream.**

It also surfaced a second airtime mismatch: **a 32 B SF7/BW250 image fragment is about 36 ms, not ≤25 ms**, so the current image fragment cap is not yet implementable with the documented fragment size.

Using the shared Python estimator in `base_station/lora_proto.py`:

| Payload | PHY | Estimated airtime |
|---|---|---:|
| 16 B cleartext `ControlFrame` | SF7/BW125/CR4-5 | 51.5 ms |
| 44 B encrypted control payload (16 B frame + 12 B nonce + 16 B tag) | SF7/BW125/CR4-5 | 92.4 ms |
| 44 B encrypted control payload | SF8/BW125/CR4-5 | 164.4 ms |
| 44 B encrypted control payload | SF9/BW125/CR4-5 | 287.7 ms |
| 44 B encrypted control payload | SF7/BW250/CR4-5 | 46.2 ms |
| 44 B encrypted control payload | SF7/BW500/CR4-5 | 23.1 ms |
| 32 B image fragment | SF7/BW250/CR4-5 | 36.0 ms |
| Max cleartext image fragment under 25 ms | SF7/BW250/CR4-5 | about 15 B |

A 20 Hz cadence has a 50 ms period. At BW125, even the unencrypted 16 B frame is already over one period; the encrypted frame is almost two periods. This means the current PHY/cadence/security combination cannot meet the plan's no-retry 20 Hz control model on one shared SX1276 channel.

The image pipeline has the same class of issue at a smaller scale: the documented 32 B fragment size misses the 25 ms P0-protection cap. Keeping the cap at SF7/BW250 means the fragment payload has to shrink to roughly 15 B, or the image PHY/cap needs to change.

## Implementation issues fixed in this pass

- `web_ui.py` now subscribes to `telemetry/#`, `status/#`, `video/#`, and `control/#`, so active-camera and source-active messages can reach the browser.
- `web_ui.py` now decodes common payload shapes: 1-byte camera/source enums, JSON link-utilization payloads, and hex fallback.
- `web_ui.py` now suppresses base joystick `ControlFrame`s when `source_active` says `handheld` or `autonomy`; E-stop remains separate and available.
- `app.js` now visually locks controls when a higher-priority source is active and surfaces link-utilization/encode-mode fields.
- Active base/tractor docs were updated away from `firmware/base_h7/base.ino`, cellular fallback, DRV8908, and SF7/BW500 references.
- Protocol tests now assert the SF7/BW125 encrypted-control airtime range so this blocker remains visible.

## Open decision before hydraulic motion

Resolve the control link mismatch before field or wet hydraulic testing. Candidate paths:

1. Return control to SF7/BW250 or BW500 for the 20 Hz control path, accepting sensitivity/FCC tradeoffs.
2. Keep SF7/BW125 but lower control cadence substantially; this likely changes operator feel and fails the current 20 Hz design assumption.
3. Reduce per-frame overhead, for example by not sending a full nonce on every frame or using a shorter authenticated tag, but only after a security review.
4. Split control onto a dedicated radio/channel and leave telemetry/image on the shared profile.
5. Bench-measure real RadioLib airtime first, but do not assume bench numbers will erase a 2x period mismatch.

Resolve the image fragment mismatch before marking the LoRa image path ready. Candidate paths:

1. Reduce image fragment cleartext payloads to roughly 15 B at SF7/BW250.
2. Use a wider image PHY profile if regulatory and range budgets allow it.
3. Revise the 25 ms cap only if the P0 starvation tests prove larger in-flight fragments are still safe.
4. Split image traffic onto a separate radio/channel in a future revision.

## Verification run needed after this note

- `python -m unittest discover -s tests` from `DESIGN-CONTROLLER/base_station`
- Python `py_compile` over `base_station/*.py`, `tools/*.py`, and tractor X8 services
- VS Code diagnostics over `base_station`, `firmware`, `tools`, and active workflow
- C/Arduino compile remains blocked until Arduino CLI/FQBN/build glue is available.
