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

1. Power Board 1 down (USB-C **and** 12V).
2. Set Max Carrier DIP switches:
   - `BOOT SEL` → **ON**
   - `BOOT`     → **ON**
3. Reconnect 12V, then USB-C.
4. Confirm SDP enumeration:
   ```powershell
   Get-PnpDevice -PresentOnly |
     Where-Object { $_.InstanceId -match 'VID_1FC9&PID_012B' } |
     Select-Object Status, FriendlyName, InstanceId | Format-List
   ```
5. Run `uuu full_image.uuu` (or equivalent OE recipe).
6. Power down. Return both DIP switches to **OFF**.
7. Power up; allow ~60 s for first-boot.
8. Run HC-01 → HC-02 → HC-04.

## Pass criteria

- SDP enumerated as `VID_1FC9&PID_012B` at step 4.
- `uuu` reports successful flash + verify.
- After reboot: HC-01..HC-04 all PASS.

## Common failure modes

| Symptom | Cause | Action |
|---|---|---|
| `1FC9:012B` does not appear with DIPs ON | DIP switches not actually flipped, carrier under-powered, or hardware fault | Re-check DIPs; T2.5 |
| `uuu` errors mid-flash | Cable / hub flakiness | Use direct host port, not a hub; retry |
| Flash succeeds but board still boots into hung state | Uncommon — image itself corrupt or pre-flash hardware fault | Try a different image build |

## Verdict matrix

| Date | Board | Outcome | Notes |
|---|---|---|---|
| (no entries yet) | | | First T3a attempt for the post-camera Board-1 wedge is pending. |
