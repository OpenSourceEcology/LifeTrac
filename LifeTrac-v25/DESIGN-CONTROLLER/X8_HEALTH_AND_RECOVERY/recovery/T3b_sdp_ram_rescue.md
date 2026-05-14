# T3b — SDP RAM-only initramfs rescue

**Tier:** 3b (diagnostic; non-destructive)

**When to try:** Before T3a, if you want to know **why** the eMMC boot is
hanging (capture `/var/log/journal`, kernel logs, OTA state). T3a wipes that
evidence.

**Why:** SDP can boot a RAM-only initramfs that mounts eMMC read-only. The
running OS never executes from the wedged eMMC, so the original boot fault
cannot recur during the rescue session.

## Pre-flight

1. T2.5 PASS (Max Carrier BMP enumerated).
2. Have a RAM-bootable initramfs image suitable for `uuu` (one of the OE
   `core-image-*` recipes works; or build a minimal Buildroot tree with
   busybox + journalctl + eMMC tools).

## Procedure

1. Same DIP-switch + power sequence as T3a (DIPs ON, expect `1FC9:012B`).
2. `uuu` boots the RAM image (no eMMC writes).
3. From the rescue shell, mount eMMC partitions read-only:
   ```sh
   mkdir -p /mnt/root /mnt/var
   mount -o ro /dev/mmcblk2p2 /mnt/root
   # /var is on the same partition by default; also mountable as overlay
   ```
4. Capture useful logs to a host-pullable location:
   ```sh
   journalctl -D /mnt/root/var/log/journal --no-pager > /tmp/journal_full.txt
   dmesg > /tmp/dmesg_rescue.txt
   ls -la /mnt/root/var/log/
   ```
5. Pull files via whatever host transport the rescue image supports (ADB if
   adbd is in the initramfs; otherwise a small TFTP/HTTP server).

## Pass criteria

- SDP enumerated.
- RAM image reaches an interactive prompt.
- eMMC mounts read-only without error.
- Journal/dmesg copied to host.

## Notes

- This is **non-destructive**. After capture, power down, return DIPs to OFF,
  power back up. eMMC contents are unchanged.
- Use this as the diagnostic step *before* T3a so you don't lose the bug.

## Verdict matrix

| Date | Board | Outcome | Notes |
|---|---|---|---|
| (no entries yet) | | | First T3b attempt pending. |
