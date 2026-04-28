# Firmware Updates

> Per [TODO.md § Operations](TODO.md). Update procedure for every node in the LifeTrac v25 stack. Each section names the artifact, the delivery channel, and the failure-recovery (rollback) path. Engineering must follow this — operators should escalate to engineering if an update notice arrives in the field.

## Trust + signing

All firmware artifacts shipped to the field are **signed** with the project's release key (Ed25519). Each node verifies the signature **before** writing the new image to its boot partition. Unsigned artifacts are refused; the previous image continues running.

The release-key public half is baked into each node's boot loader at manufacture. Rotation requires shipping a new boot loader (rare; treated as a hardware-recall-tier event).

## Per-node update path

### Tractor — Portenta H747 (M7 + M4 firmware)

- **Artifact:** `lifetrac-tractor-h747-vX.Y.Z.bin` (signed flat image; M7 + M4 are linked together).
- **Channel:** USB-C from a maintenance laptop running `tools/flash_tractor.py`. **Never OTA** — the tractor radio runs control-class traffic and we don't risk a half-flashed M7 in the field.
- **Procedure:**
  1. Park the machine, key-off, chock wheels.
  2. Connect USB-C to the H747 from the maintenance laptop.
  3. `python tools/flash_tractor.py --image lifetrac-tractor-h747-vX.Y.Z.bin --port COM5`.
  4. The tool: (a) verifies signature, (b) sends the H747 into bootloader mode, (c) writes flash, (d) verifies readback CRC, (e) reboots, (f) reads back firmware version over USB-CDC. Exit 0 = success.
  5. Walk through the daily safety check from [OPERATIONS_MANUAL.md § 8](OPERATIONS_MANUAL.md).
- **Rollback:** `tools/flash_tractor.py --image lifetrac-tractor-h747-v<previous>.bin` — the H747 keeps the previous image archived in `/lifetrac/firmware-archive/` on the maintenance laptop. (We don't do dual-bank A/B on the H747; the bootloader is single-image because flash is at a premium with the full M7+M4 link.)

### Tractor — Portenta X8 (Linux side, image_pipeline + camera_service + bridge)

- **Artifact:** OCI image `ghcr.io/openmanufacturing/lifetrac-tractor-x8:vX.Y.Z`, plus a Yocto-side base image refresh `lifetrac-x8-base-vA.B.bin` only on a major bump.
- **Channel:** OTA over the base-station's Wi-Fi. The X8 pulls the OCI image; the base only fetches once and caches.
- **Procedure (operator-side maintenance window):**
  1. Engineering pushes the new image to GHCR and updates `config/x8_image_tag` on the base.
  2. The base sends `CMD_UPDATE_AVAILABLE` (priority P1, opcode TBD — currently piggybacks on the params channel) to the X8.
  3. Operator gets a UI banner `Update available — apply during next idle window`. The operator presses APPLY when the machine is parked.
  4. The X8 docker-pulls the new tag, verifies its signature against the cached release key, performs `docker compose up -d` to restart the affected services. Camera + image_pipeline drop offline for ~30 s.
- **Rollback:** the previous tag stays in the local docker cache for 7 days. `docker compose up -d --pull=never lifetrac-x8:vX.Y.<Z-1>` restores the previous version.

### Base station — Portenta H747 (radio gateway M7 firmware)

- **Artifact:** `lifetrac-base-h747-vX.Y.Z.bin`.
- **Channel:** USB-C from the base-station laptop / X8 directly via USB-CDC.
- **Procedure:** identical to the tractor H747 path, but the device path is the base's H747 (USB-CDC enumerates as a separate ttyACM* from the X8 Linux side).
- **Rollback:** identical.

### Base station — Portenta X8 (`web_ui.py`, `lora_bridge.py`, `image_pipeline/*`)

- **Artifact:** OCI image `ghcr.io/openmanufacturing/lifetrac-base-x8:vX.Y.Z`.
- **Channel:** OTA over LAN; the X8 pulls from GHCR.
- **Procedure:**
  1. SSH to the X8 (`ssh fio@<base-ip>`).
  2. `sudo /usr/local/bin/lifetrac-update vX.Y.Z` — wraps the docker-pull / verify-signature / `docker compose up -d` sequence.
  3. Watch `journalctl -fu lifetrac-base` for ≥1 minute to confirm the new bridge handshakes with the tractor.
- **Rollback:** `sudo /usr/local/bin/lifetrac-update vX.Y.<Z-1>` — same wrapper, previous tag.

### Handheld — Portenta H747 (M7 firmware) + secure element

- **Artifact:** `lifetrac-handheld-h747-vX.Y.Z.bin` for application code; secure-element provisioning is **separate** and only happens at manufacture or pair.
- **Channel:** USB-C from the base-station laptop. **Never OTA** — handheld is the operator's safety-critical surface.
- **Procedure:**
  1. Connect handheld via USB-C while it is **powered off**. The bootloader enumerates as `Arduino_Bootloader` on USB.
  2. `python tools/flash_handheld.py --image lifetrac-handheld-h747-vX.Y.Z.bin --port COM7`.
  3. Tool verifies signature, flashes, reboots. Power up the handheld and confirm the OLED displays the new firmware version on the splash screen.
  4. **Re-run pairing** — the secure-element key is preserved across firmware updates by design, but verify pairing still works (cf. [OPERATIONS_MANUAL.md § 2](OPERATIONS_MANUAL.md)).
- **Rollback:** keep the previous `.bin` on the maintenance laptop; reflash with the previous version via the same procedure.

## Update sequencing across the fleet

When rolling a multi-node release:

1. Update the **base-station X8** first (lowest blast radius — operator can roll it back fastest).
2. Update **base-station H747** next.
3. Update **tractor X8** during a maintenance window when the operator can spare 30 s of camera offline.
4. Update **tractor H747** last (highest risk — but updating any earlier risks a protocol mismatch with an unmodified base).
5. Update **handheld** any time after the base is on the new version.

If steps 1–2 succeed and step 3 or 4 fails, the previous-version base/X8 will still talk to the previous-version tractor — protocols are versioned per `LORA_PROTOCOL.md`.

## Testing before a release ships

Engineering's pre-release gates (run by CI before tagging a `vX.Y.Z`):

- All `base_station/tests/*` pass (≥ 71 tests at time of writing).
- `LORA_PROTOCOL.md` version field bumped if any wire format changes.
- `CYBERSECURITY_CASE.md` reviewed for any new privileged surface.
- A bench-rig dry run: tractor + base + handheld talking for ≥30 minutes with the new firmware on all three, including a manual E-stop and a pairing cycle.

## See also

- [CYBERSECURITY_CASE.md](CYBERSECURITY_CASE.md) — signing key custody
- [BASE_STATION.md](BASE_STATION.md), [TRACTOR_NODE.md](TRACTOR_NODE.md), [HANDHELD_REMOTE.md](HANDHELD_REMOTE.md)
- [tools/pair_handheld.py](tools/pair_handheld.py)
