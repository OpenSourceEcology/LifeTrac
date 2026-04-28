# LifeTrac v25 OTA Strategy

**Status:** DRAFT FOR REVIEW. Defines the firmware-update story for the
H7 (M7+M4), Opta, and X8 Linux sidecar. Hardware-validated rollback
remains a bench-test work item (see SAFETY_CASE.md §7).

**Companions:** [KEY_ROTATION.md](KEY_ROTATION.md), [SAFETY_CASE.md](SAFETY_CASE.md),
[CYBERSECURITY_CASE.md](CYBERSECURITY_CASE.md), [MASTER_PLAN.md](MASTER_PLAN.md).

## 1. Targets

| Target | What ships | Storage | Update channel |
|--------|------------|---------|----------------|
| Portenta H747 / M7 | tractor_m7.ino | Internal flash bank A/B | USB-DFU on bench; over-the-LAN deferred to v26 |
| Portenta H747 / M4 | tractor_m4.cpp | Internal flash, alongside M7 | Same image as M7 (mbed dual-core build) |
| Arduino Opta | opta_modbus_slave.ino | Internal flash | USB-CDC, manual |
| Portenta X8 Linux | base_station/, firmware/tractor_x8/ | OSTree-managed Yocto | apt-style repository on the install bench |

v25 explicitly ships **wired-only OTA** — over-the-air firmware push to a
field tractor is **out of scope**. The decision is documented in
[DECISIONS.md](DECISIONS.md) §D-OTA-1.

## 2. Signing

All H7, M4, and Opta firmware images are signed with the same RSA-3072
key pair as the install bench keystore:

- Private key: `tools/keys/firmware_signing.pem` (offline, install-bench only)
- Public key: embedded in the bootloader of each MCU at first provision

The bootloader (mbed-bootloader on H7; stock Opta bootloader for now)
verifies the signature **before** activating the new bank. An unsigned or
mis-signed image is rejected and the previous bank stays active.

X8 Linux images use the upstream OSTree GPG signature with the same
fleet key.

## 3. Update flow (H7 example)

```
[bench operator] arduino-cli compile + upload
         │
         ▼  USB-DFU
[H7 bootloader] receives image into bank B
         │
         ▼  validate
   signature ok? ── no ──► reject, beep, stay on bank A
         │
         ▼  yes
   write "boot bank B once" flag, reset
         │
         ▼
[bank B firmware boots]
   first 60 s:  health check
     - M7 main loop reaches steady state?
     - M4 watchdog arms (PIN_PSR_ALIVE goes high)?
     - Opta Modbus reads (REG_FW_VERSION) succeed?
     - At least one TelemetryFrame TX'd successfully?
         │
         ▼  if all yes within 60 s
   bootloader marks bank B as the new "good" bank
         │
         ▼  if any check fails
   reset → bootloader reverts to bank A
```

The "boot bank B **once**" flag is critical — if the new image hangs
before clearing it, the next reset goes back to bank A automatically.
This is the standard A/B + watchdog rollback pattern.

## 4. Update flow (X8 Linux)

OSTree-style atomic deploy:

1. `ostree pull` from the install-bench HTTP repo.
2. `ostree admin deploy` stages the new tree at next boot.
3. Reboot into the new tree.
4. `lifetrac-health` systemd unit runs for 5 minutes; on success it
   marks the deploy as committed. On failure (or no-success after
   5 minutes) the next reboot rolls back to the prior tree.

The `lifetrac-health` unit checks:

- `lora_bridge.service` active and not flapping
- `web_ui.service` reachable on `127.0.0.1:8080/api/session`
- mosquitto reachable
- audit_log appending (file size grew since boot)

## 5. Versioning

- **MCU firmware**: `MAJOR.MINOR` BCD in `REG_FW_VERSION` (Opta) and the
  matching constant in `tractor_m7.ino`. Bumped manually by the
  bench operator before signing.
- **X8**: semantic-version git tag, embedded in `/etc/lifetrac-version`.
- **Compatibility matrix**: tracked in `MASTER_PLAN.md` §8.21 (planned).
  Each release lists the (M7, M4, Opta, X8) tuple known to work
  together.

## 6. Rollback test (bench validation)

Before each release the install bench shall:

1. Flash a deliberately-broken M7 image (e.g. `setup()` returns immediately).
2. Confirm the H7 bootloader reverts to the previous bank within ~70 s.
3. Confirm the M4 watchdog held the PSR safety relay open the entire time.
4. Confirm the X8 audit log captured the rollback as a `boot_failed` event.

Steps 1–4 are documented in `BUILD-CONTROLLER/` once that directory is
populated.

## 7. Known limitations

- **No remote field OTA** — every update needs USB or LAN access to the
  machine.
- **Single signing key** — compromise means re-keying every node. Same
  mitigation as KEY_ROTATION.md: bench-side rotation, pre-shared during
  a service visit.
- **No delta updates** — full image flash every time. Acceptable on USB.
