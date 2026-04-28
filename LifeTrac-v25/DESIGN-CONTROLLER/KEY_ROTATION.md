# LifeTrac v25 — Key Rotation Procedure

**Audience:** the operator who owns the base station + tractor + handheld.
**Companion to:** `MASTER_PLAN.md §8.6` (single-key, build-time provisioning) and `tools/provision.py` (USB-CDC key writer; future).

The v25 air interface uses a single 16-byte AES-128-GCM pre-shared "fleet key" baked into firmware on every node (handheld, tractor M7, base-station Python). Rotating the key is a manual, physically-attended procedure. It is **not** done over the air.

---

## When to rotate

- The key was committed to a public repo by accident.
- A node was lost / stolen / sold.
- Annual hygiene rotation (recommended).
- A new operator joins and you want a clean cutover.

Routine field reboots **do not** require a new key.

---

## What you need

- USB-C cable for each node.
- A workstation with the `DESIGN-CONTROLLER/` checkout, `arduino-cli`, and `python3 + cryptography`.
- `openssl` (or any 32-hex-char generator).
- Physical access to **all three** of: handheld, tractor enclosure, base-station enclosure. **All-or-nothing.** A node still running the old key cannot talk to the rotated fleet.

---

## Procedure

1. **Park the tractor**, set parking brake, hardware E-stop pressed.

2. **Generate the new key** on the workstation:

    ```bash
    openssl rand -hex 16
    # e.g. 7f9c4e1a2d8b063f5a1e9d8c7b6a5f4e
    ```

    Treat this string the same way you treat your WiFi password. Don't paste it into chat / pastebin / screenshare.

3. **Update the firmware key file:**

    ```bash
    cd DESIGN-CONTROLLER/firmware/common/lora_proto
    cp key.h.example key.h          # if first time
    # edit key.h, replace the 16-byte LP_FLEET_KEY array with your new bytes
    ```

    `key.h` is `.gitignore`d; never commit it.

4. **Update the base-station key.** The Python bridge currently reads `FLEET_KEY = bytes(16)` from `base_station/lora_bridge.py`. Replace with your new bytes (or, preferred, set `LIFETRAC_FLEET_KEY_HEX` in the base-station systemd unit and have `lora_bridge.py` read it from the env). Restart `lora_bridge.service`.

5. **Reflash all three Arduino targets.** Order doesn't matter as long as all three are done before you re-power the tractor:

    ```bash
    arduino-cli compile --fqbn arduino:mbed_portenta:envie_m7 firmware/tractor_h7
    arduino-cli upload  --fqbn arduino:mbed_portenta:envie_m7 -p COM3 firmware/tractor_h7

    arduino-cli compile --fqbn arduino:samd:mkrwan1310 firmware/handheld_mkr
    arduino-cli upload  --fqbn arduino:samd:mkrwan1310 -p COM4 firmware/handheld_mkr

    arduino-cli compile --fqbn arduino:mbed_opta:opta firmware/tractor_opta
    arduino-cli upload  --fqbn arduino:mbed_opta:opta -p COM5 firmware/tractor_opta
    ```

    Production builds **must** include `-DLIFETRAC_USE_REAL_CRYPTO`; never ship `-DLIFETRAC_ALLOW_STUB_CRYPTO` to a tractor.

6. **Verify the link** with the tractor on jacks (wheels off the ground), parking brake on:

    - Power up the tractor.
    - Power up the base, watch `journalctl -u lora_bridge -f`. You should see `bridge_start` then `rx` events for tractor heartbeats.
    - Power up the handheld. The OLED should show RSSI within ~10 s.
    - Send a `CMD_ESTOP` from each source (handheld red mushroom, base UI big red button) and confirm the audit log records all three.

7. **Bench-test E-stop** before driving:

    - Press handheld mushroom → tractor LEDs / valves should drop within 250 ms.
    - Click base-UI E-STOP → same.
    - Pull the hardware PSR button on the base → same.
    - Cut LoRa entirely (power off base + handheld) → tractor watchdog must trip valves within 200 ms (Opta safety state machine).

8. **Destroy the old key.** Delete it from your password manager / paper notebook. The new key is now the only valid key for this fleet.

---

## What goes wrong

| Symptom | Likely cause | Fix |
|---|---|---|
| `gcm_tag_reject` events flood the audit log on a single node | That node still has the old key | Reflash it. |
| Tractor watchdog trips immediately on first power-up | Opta still has stub crypto and bridge has real crypto (or vice-versa) | Confirm `-DLIFETRAC_USE_REAL_CRYPTO` on both Arduino targets. |
| Handheld OLED shows "no link" forever | Wrong key, wrong region (`#define LIFETRAC_REGION_EU` vs default US), or wrong PHY profile | Reflash with matching `key.h` + region defines. |
| Base-UI login fails after rotation | Unrelated — that's the PIN, not the AES key. PIN lives in the systemd unit env (`LIFETRAC_PIN`). | Re-set `LIFETRAC_PIN` and restart `web_ui.service`. |

---

## Future hardening (out of v25 scope)

Per `MASTER_PLAN.md §8.6`, these belong in `RESEARCH-CONTROLLER/`:

- Per-device keys with a key server.
- Secure-element storage (ATECC608A) so the key never sits in flash.
- On-air key handshake with forward secrecy.
- Automated rotation cadence + alerting.
