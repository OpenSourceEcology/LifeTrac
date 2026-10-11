# T3a — SDP / `uuu` full image reflash

**Tier:** 3a (definitive recovery; bypasses eMMC entirely)

**When to try:**
- T2 cold power cycle did not recover (boot itself wedges).
- Suspected eMMC corruption (rootfs or OTA partitions damaged).
- Want a known-good baseline before re-running diagnostics.
- Bringing up a new bench board: this is step 4 of
  [BENCH_SETUP.md](../../firmware/x8_lora_bootloader_helper/bench_tools/BENCH_SETUP.md).

**Why it always works:** SDP (Serial Download Protocol) is in i.MX8MM mask
ROM, not eMMC, so it runs even if the entire on-disk OS is destroyed. `uuu`
walks the SDP USB endpoint and re-images eMMC end-to-end.

**What it erases:** the whole eMMC: OS, `/etc` changes, docker images,
`/home/fio` and the base's deployed tree and `secrets/`. Capture first if
anything matters (`bench_tools/pull_board_state.sh`). It does **not** touch
the L072 firmware, which lives on the carrier's Murata module. After a
reflash, run the board through BENCH_SETUP steps 5–7 again (first boot,
`provision_bench_board.sh`, verify), then redeploy.

## Image and tool (pinned)

### Which image

| image | kernel | URL | on this bench |
|---|---|---|---|
| **LmP 4.0.11-934-91** | 6.1.24 | `https://downloads.arduino.cc/portentax8image/934.tar.gz`: 1,097,240,589 B, Last-Modified 2025-08-11 | The base (`2D0A1209DABC240B`), flashed 2026-05-13 and 2026-05-24. **Use this one.** |
| LmP 4.0.3-674-88 | 5.10.93 | `https://downloads.arduino.cc/portentax8image/674.tar.gz`: 854,998,055 B | The tractor's (`2E2C1209DABC240B`) factory image; that board was never reflashed. Hosted, but never downloaded or byte-verified here, so there is no sha256 on record. |
| `image-latest.tar.gz` | — | `https://downloads.arduino.cc/portentax8image/image-latest.tar.gz` | **Not the bench image.** The URL moves to newer builds. On 2026-05-08 it still served 934, and the bench bundle was downloaded under that name. |

### Download and verify (Git Bash)

```bash
mkdir -p ~/portenta-x8-reflash && cd ~/portenta-x8-reflash
curl -fLO https://downloads.arduino.cc/portentax8image/934.tar.gz
sha256sum 934.tar.gz
tar xzf 934.tar.gz                                # creates 934/
cd 934
tar xzf mfgtool-files-portenta-x8.tar.gz          # creates mfgtool-files-portenta-x8/
gunzip -k lmp-factory-image-portenta-x8.wic.gz    # 3,222,240,256 B .wic; -k keeps the .gz
cd ..
sha256sum -c 934.sha256                           # the list below, saved as 934.sha256
```

The bench bundle's outer tarball, saved as `image-latest.tar.gz` on
2026-05-08, has sha256
`2f2065c7b10ce33d8444a138353187c507727b5109ddca325506ecaea1efe221`. Today's
`934.tar.gz` has the same size and Last-Modified date. Nobody has compared
it byte for byte, so it is expected to match but not proven. If your hash
differs, the files inside still decide. Save this list as `934.sha256` next
to the `934/` folder; it holds the sha256 of every file in the bundle used
for both base reflashes, computed 2026-10-04:

```text
9cd4dbf7acdb4c4737cc50ad2d23d7deeb136920d37ba42f6ac064b5aaa531df *./934/imx-boot-portenta-x8
8bc5938a61418c02665baf0d721800045c2caf9818ff1e2c2ab33ca44cad9cc7 *./934/lmp-factory-image-portenta-x8.wic
a9571a611922170ea7a350a250abd45493652e21393ec6a6806ca1365da73df0 *./934/lmp-factory-image-portenta-x8.wic.gz
ddeaa6f239feb5cfe9b88ae67a2fe8ef586d52faa8dad609320e338ed596ab2a *./934/mfgtool-files-portenta-x8.tar.gz
fbb6e417a215d1dfc07de6f069d3b65b7621efa7f1c33fda4d00148e99782421 *./934/mfgtool-files-portenta-x8/bootloader.uuu
339e63cb0013b8876df643c163440f04fcc2b2a4d4571257660e6d5f25b1936a *./934/mfgtool-files-portenta-x8/erase_emmc.uuu
e624aa9720debc2914c11d598cbaf5b6b965e7865ea8bdfb3031db7de6b7f4a6 *./934/mfgtool-files-portenta-x8/fitImage-portenta-x8-mfgtool
3ff0a055c799eb1e5d0c1e4af870d764c968bff0ebbd21c8b6b2da1c15e49af7 *./934/mfgtool-files-portenta-x8/full_image.uuu
f97e3afaa8491ce4b7023470975d5639afd5302ac79db00c92f508148d9fc300 *./934/mfgtool-files-portenta-x8/imx-boot-mfgtool
a63a73ec50c455dc4666df3b4bc5161ffcbd2c9e9201638a6327fba81c5a582a *./934/mfgtool-files-portenta-x8/linux_initramfs.uuu
01e04a0a0e85a99ee0050c7377ca9bf83048a0612409be233b61771eabac3212 *./934/mfgtool-files-portenta-x8/probe_emmc.uuu
a4f945f5e81255fe52a97143424274e50da01358c83df55ef8a1b02e18a2da4a *./934/mfgtool-files-portenta-x8/probe_sdcard.uuu
20d6aa39aa525d6325e879853bc40fd0ed751da8aacfc60bd9025de0de7d9bc5 *./934/mfgtool-files-portenta-x8/test_ram.uuu
51d4f5f95316e3a5026e988f255200f0d7862f97bd9e1063273a37e4b7a50b51 *./934/mfgtool-files-portenta-x8/u-boot-mfgtool.itb
d5568f967a686dcef4f60932baba2dc8393b79e43dab5c1437226b2a34658a40 *./934/mfgtool-files-portenta-x8/uuu
ae5dd2b1a6575d1a2416a51ab849009bcaf230d803692c46ade95266b6dcc08e *./934/mfgtool-files-portenta-x8/uuu.exe
2e37d04e7e4435e25dbe0f9463efe672324e1718b0a272b2d6627b90e13cb4fa *./934/mfgtool-files-portenta-x8/uuu_mac
c8afc30f003ec18c278ef8da3307f075ffad0bd744565feb95e09f2e8f3d16da *./934/sit-portenta-x8.bin
b4c055ed4fe6aa7f553fdf2cdcefbfea528d5917abf740a8b55510b52682af08 *./934/u-boot-portenta-x8.itb
```

The source of this list is
`LifeTrac-bench-archive\board_state_2026-10-04\pc_only\lmp_934_bundle_sha256.txt`
in the private archive (PC only).

### `uuu` version: 1.5.243

Use **`uuu` 1.5.243** from NXP's mfgtools releases
(<https://github.com/nxp-imx/mfgtools/releases>, tag `uuu_1.5.243`). The
bench's copy of `uuu.exe` has sha256
`f6b76a6246befabeadfebdc1cbfe58f35939596caf7b78717ceab599b0c85027`.
**Do not use the `uuu.exe` inside `mfgtool-files-portenta-x8/`.** That one
is 1.5.109 (sha256 `ae5dd2b1…`), and it was unstable on Windows 11 with this
image. Both bench reflashes ran 1.5.243 (12/12 stages).

### Extracted layout

`full_image.uuu` refers to the image files as `../<file>`, so run `uuu` from
inside `mfgtool-files-portenta-x8/`:

```text
portenta-x8-reflash/
├── 934.tar.gz
├── 934.sha256
└── 934/
    ├── imx-boot-portenta-x8
    ├── lmp-factory-image-portenta-x8.wic        (from the .wic.gz)
    ├── lmp-factory-image-portenta-x8.wic.gz
    ├── mfgtool-files-portenta-x8.tar.gz
    ├── mfgtool-files-portenta-x8/               (from the .tar.gz)
    │   ├── full_image.uuu   <- run uuu here
    │   ├── imx-boot-mfgtool, u-boot-mfgtool.itb, fitImage-portenta-x8-mfgtool
    │   └── ... (other .uuu scripts; bundled uuu 1.5.109, do not use)
    ├── sit-portenta-x8.bin
    └── u-boot-portenta-x8.itb
```

## Pre-flight

1. Run T2.5 first: confirm `VID_1366&PID_0105` (the Max Carrier's on-board
   J-Link) is enumerating. If not, the carrier is unpowered and SDP won't
   appear either.
2. Have the image downloaded, verified and extracted as above, and
   `uuu` 1.5.243.
3. Use a USB-A to USB-C data cable into the **X8 module's** USB-C port,
   plugged straight into the PC. No hub, and no USB-C to USB-C cable.

## Procedure

1. Power the board down (USB-C **and** 12 V).
2. Set the Max Carrier DIP switches: **`BOOT SEL` ON** (`BOOT` OFF). If SDP
   does not appear at step 5, power down and **add `BOOT` ON**.
   - Both reflashes on this bench, `2D0A1209DABC240B` on 2026-05-13 and
     2026-05-24, entered SDP with `BOOT SEL` ON only.
   - `BOOT SEL` + `BOOT` both ON is what Arduino's uuu guide and the
     2026-05-12 recovery plan describe. It was never needed here.
3. Start `uuu` so it is waiting, in PowerShell:
   `cd ...\934\mfgtool-files-portenta-x8; & <path>\uuu.exe full_image.uuu`.
   It prints `Waiting for Known USB Device Appear...`.
4. Power up: connect 12 V, then USB-C. On 2026-05-13 SDP appeared on USB-C
   alone; on 2026-05-24 it needed USB-C and 12 V. Connect both.
5. Confirm SDP enumeration, in another PowerShell window:
   ```powershell
   Get-PnpDevice -PresentOnly |
     Where-Object { $_.InstanceId -match 'VID_1FC9&PID_(012B|0134)' } |
     Select-Object Status, FriendlyName, InstanceId | Format-List
   ```
   Both bench reflashes showed `PID_0134`. `PID_012B` is the PID that older
   i.MX mask-ROM revisions use; it has not been seen on this bench.
   `uuu` 1.5.243 accepts both.
6. `uuu` pushes the boot loaders, then writes the full wic image. That takes
   about 60–80 minutes (≈75 min on 2026-05-24). Success is 12/12 stages
   and `Success 1 Failure 0`.
7. Power down and set **both DIP switches OFF**.
8. First boot (next section).
9. Run HC-01, then HC-02. Run HC-04 only for a flash-pipeline session.

## First boot

1. With both DIP switches OFF, connect 12 V, then USB-C. Allow about 60 s.
   If the LEDs go red or adb has not appeared after 2 minutes, power-cycle
   once more; the 2026-05-13 reflash needed two cycles.
2. `adb devices -l` must list the serial as `device`. Then
   `adb -s <serial> shell cat /etc/os-release` must show
   `VERSION="4.0.11-934-91"`, and `uname -r` must show `6.1.24-lmp-standard`.
3. Continue with BENCH_SETUP step 5: password, sudoers drop-in, base
   network and ssh key, then `provision_bench_board.sh`.

> **First-boot caveat (LmP 934-91):** the freshly imaged X8 has no Wi-Fi
> credentials and no SSH password. The only post-flash channel is `adb` over
> USB-C. **Do NOT run `adb kill-server`**: first-boot adbd is fragile, and a
> kill-server leaves the board adb-invisible until a USB-C unplug and replug.
> See `T0_adb_daemon_kick.md` and the `lifetrac-x8-adb-reverse-broken` repo
> memory.

## Pass criteria

- SDP enumerated as `VID_1FC9&PID_0134` (or `VID_1FC9&PID_012B` on older
  silicon) at step 5.
- `uuu` reports successful flash and verify (full wic write ≈ 60–80 min).
- After reboot: `os-release` shows the pinned version, and HC-01 and HC-02
  PASS.

## Common failure modes

| Symptom | Cause | Action |
|---|---|---|
| No `1FC9:0134` (or `012B`) with `BOOT SEL` ON | DIP switches not actually flipped, `BOOT` also needed, carrier under-powered, or a hardware fault | Re-check the DIP switches; add `BOOT` ON; connect 12 V as well as USB-C; T2.5 |
| `uuu` errors mid-flash | Cable or hub flakiness, or the bundled `uuu` 1.5.109 | Use a direct host port, not a hub; use `uuu` 1.5.243; retry |
| `full_image.uuu` cannot find the `.wic` | `.wic.gz` not decompressed, or `uuu` not run from `mfgtool-files-portenta-x8/` | See *Extracted layout* |
| Flash succeeds, but the board boots a different LmP version | Wrong bundle (for example `image-latest.tar.gz`) | Check `/etc/os-release`; reflash with the pinned bundle |
| Flash succeeds but board still boots into hung state | Uncommon: the image itself is corrupt, or a pre-flash hardware fault | Verify the sha256 list; try again |

## Verdict matrix

The May notes call `2D0A1209DABC240B` "Board 1" on 2026-05-13 and "Board-2"
on 2026-05-24. Go by the serial.

| Date | Board | Outcome | Notes |
|---|---|---|---|
| 2026-05-13 | `2D0A1209DABC240B` (then "Board 1") | PASS | DIP = `BOOT SEL` ON **only**. USB-C only. SDP enumerated as `VID_1FC9&PID_0134`. `uuu` 1.5.243 ran `full_image.uuu`: 12/12 stages, Success 1 / Failure 0. First boot needed 2 power cycles. Fresh `4.0.11-934-91` / `6.1.24-lmp-standard`. Source: [log/2026-05-13_board1_T3a_postreflash_v1.md](../log/2026-05-13_board1_T3a_postreflash_v1.md). *Corrected 2026-10-10 against that log. The row used to say "Board-1 (2E2C1209DABC240B)", "`BOOT`+`BOOT SEL` both ON" and "`PID_012B`". `2E2C` (the tractor) has never been reflashed.* |
| 2026-05-24 | Board-2 (2D0A1209DABC240B) | PASS | DIP = `BOOT SEL` ON **only** (`BOOT` OFF). SDP enumerated as `VID_1FC9&PID_0134` (NOT `012B`). USB-C + 12V both required to enter SDP. `uuu` 1.5.243 EXIT=0; full wic write ≈75 min total. First-boot adbd later wedged after host-side `adb kill-server`; recovery requires USB-C unplug/replug (no software-only path on a factory image with no Wi-Fi/SSH). |
