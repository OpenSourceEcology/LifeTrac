# T3a — SDP / `uuu` full image reflash

**Tier:** 3a (definitive recovery; bypasses eMMC entirely)

**When to try:**
- T2 cold power cycle did not recover (boot itself wedges).
- Suspected eMMC corruption (rootfs or OTA partitions damaged).
- Want a known-good baseline before re-running diagnostics.

**Why it always works:** SDP (Serial Download Protocol) is in i.MX8MM mask
ROM, not eMMC, so it runs even if the entire on-disk OS is destroyed. `uuu`
walks the SDP USB endpoint and re-images eMMC end-to-end.

## Pre-flight

1. Run T2.5 first — confirm `VID_1366&PID_0105` (Max Carrier BMP) is
   enumerating. If not, the carrier is unpowered and SDP won't appear either.
2. Have a known-good `full_image.uuu` and image set on the host.

## Procedure

1. Power Board down (USB-C **and** 12V).
2. Set Max Carrier DIP switches. **The required position is board-dependent**
   — try (a) first; if SDP does not enumerate after a 12V+USB-C re-attach,
   power down and try (b):
   - (a) `BOOT SEL` → **ON** only (`BOOT` → OFF)  *— worked on Board-2 (2D0A) 2026-05-24*
   - (b) `BOOT SEL` → **ON** and `BOOT` → **ON**  *— worked on Board-1 2026-05-13*
3. Reconnect 12V, then USB-C.
4. Confirm SDP enumeration. The mask-ROM PID also varies between silicon revisions:
   ```powershell
   Get-PnpDevice -PresentOnly |
     Where-Object { $_.InstanceId -match 'VID_1FC9&PID_(012B|0134)' } |
     Select-Object Status, FriendlyName, InstanceId | Format-List
   ```
   Observed PIDs: `012B` (Board-1, 2026-05-13) and `0134` (Board-2, 2026-05-24).
   Either is a valid i.MX SDP endpoint and `uuu` 1.5.243+ accepts both.
5. Run `uuu full_image.uuu` (or equivalent OE recipe).
6. Power down. Return both DIP switches to **OFF**.
7. Power up; allow ~60 s for first-boot.
8. Run HC-01 → HC-02 → HC-04.

## Pass criteria

- SDP enumerated as `VID_1FC9&PID_012B` **or** `VID_1FC9&PID_0134` at step 4.
- `uuu` reports successful flash + verify (full wic write ≈ 60–80 min).
- After reboot: HC-01..HC-04 all PASS.

> **First-boot caveat (LmP 934-91):** the freshly imaged X8 has no Wi-Fi
> credentials and no SSH password. The only post-flash channel is `adb` over
> USB-C. **Do NOT run `adb kill-server`** — first-boot adbd is fragile and a
> kill-server leaves the board adb-invisible until a USB-C unplug/replug. See
> `T0_adb_daemon_kick.md` and the `lifetrac-x8-adb-reverse-broken` repo memory.

## Common failure modes

| Symptom | Cause | Action |
|---|---|---|
| `1FC9:012B` does not appear with DIPs ON | DIP switches not actually flipped, carrier under-powered, or hardware fault | Re-check DIPs; T2.5 |
| `uuu` errors mid-flash | Cable / hub flakiness | Use direct host port, not a hub; retry |
| Flash succeeds but board still boots into hung state | Uncommon — image itself corrupt or pre-flash hardware fault | Try a different image build |

## Verdict matrix

| Date | Board | Outcome | Notes |
|---|---|---|---|
| 2026-05-13 | Board-1 (2E2C1209DABC240B) | PASS | DIP = `BOOT`+`BOOT SEL` both ON. SDP enumerated as `VID_1FC9&PID_012B`. `uuu` 1.5.243 EXIT=0. |
| 2026-05-24 | Board-2 (2D0A1209DABC240B) | PASS | DIP = `BOOT SEL` ON **only** (`BOOT` OFF). SDP enumerated as `VID_1FC9&PID_0134` (NOT `012B`). USB-C + 12V both required to enter SDP. `uuu` 1.5.243 EXIT=0; full wic write ≈75 min total. First-boot adbd later wedged after host-side `adb kill-server`; recovery requires USB-C unplug/replug (no software-only path on a factory image with no Wi-Fi/SSH). |
