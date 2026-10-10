# Bench PC setup (Windows 11)

One Windows 11 PC drives the bench. Use **Git Bash** for the board scripts
(`*.sh`, adb, ssh) and **PowerShell** for the radio harness and
`mingw32-make`. The versions below are the ones that produced the RS-12 and
RS-13 evidence. Where a version is marked exact, a different one gives
different bytes. The PC needs no Docker, because images are built on the base
X8 ([DEPLOY.md](DEPLOY.md)). Next step: [BENCH_QUICKSTART.md](BENCH_QUICKSTART.md).

| tool | bench PC version | needed for |
|---|---|---|
| Git for Windows (Git Bash) | 2.52 | checkout, board scripts |
| Python with the `py` launcher | 3.14 (CI: 3.11) | harness helpers, analysis tools, tests |
| Arm GNU Toolchain | **12.2.MPACBTI-Rel1, exact** | byte-identical L072 builds |
| WinLibs `mingw32-make` | GNU Make 4.4.1 | L072 build (PowerShell only) |
| Android platform-tools (`adb`) | current | tractor access, programming |
| `uuu` | 1.5.243 | OS reflash only |
| arduino-cli + `arduino:mbed_portenta` | 1.4.1 + **4.5.0** | optional: X8 H7/M4 sketches |
| SEGGER J-Link Software | current | optional: last-resort L072 recovery |
| Firefox | current | the moving scene for camera legs |

## Git for Windows

```bash
winget install Git.Git
git clone -c core.longpaths=true https://github.com/OpenSourceEcology/LifeTrac.git
```

- **Long paths.** Some `bench-evidence/` paths are longer than Windows' 260
  characters, and the checkout fails on them without `core.longpaths`. For an
  existing clone, run `git config core.longpaths true`.
- **Line endings.** Git for Windows defaults to `core.autocrlf=true`. The
  repo's [`.gitattributes`](../../../../../.gitattributes) keeps board-side
  files (shell scripts, openocd `.cfg`, ...) LF in the working tree, because
  a CRLF script breaks on the board: you get `1: image` from the flash
  wrapper, or `/tmp/lifetrac_p0c\r/...`. Files checked out before
  `.gitattributes` landed stay CRLF until git rewrites them, so still strip
  `\r` when you push ([FLASH_RUNBOOK.md](../FLASH_RUNBOOK.md) §1).
- **ssh** ships with Git for Windows. Create the bench key once, then install
  its public half on the base as [BENCH_SETUP.md](BENCH_SETUP.md) says. Keep
  the private key in a password manager, never in git.

  ```bash
  ssh-keygen -t ed25519 -f ~/.ssh/lifetrac_base_ed25519 -C lifetrac-bench-pc
  ```

## Python

Install Python 3.11 or newer from python.org; it comes with the `py`
launcher. Every command in the kit uses `py -3`. From the repo root:

```bash
py -3 -m pip install -r LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/bench_tools/requirements-pc.txt
py -3 -m pip install -r LifeTrac-v25/DESIGN-CONTROLLER/base_station/requirements-dev.txt
```

- [`requirements-pc.txt`](requirements-pc.txt) holds what the harness and the
  PC-side bench tools import, including the amqtt broker.
- `requirements-dev.txt` holds the test suite and the base-station modules
  that the analysis tools load (`tools/vector_dry_run.py replay`,
  `tools/rs12_leg_report.py`).

The harness starts a PC MQTT broker (`start_mqtt_broker.py` at the repo root,
amqtt, listening on `0.0.0.0:1883`) when nothing answers on `-HostIp`:1883.

- Check that the harness finds your Python. Older copies hardcoded the
  original bench PC's `python.exe` path.
- The base's rx daemon uses this broker as its control broker, so allow
  inbound TCP 1883 when Windows Defender Firewall asks, on **Private**
  networks only.
- Retained control topics live on this broker as well as on the base's.
  Clear both between campaigns with `clear_retained_host.py`
  ([BENCH_RUNBOOK.md](BENCH_RUNBOOK.md)).

### Run the test suite locally

```bash
cd LifeTrac-v25/DESIGN-CONTROLLER/base_station
LIFETRAC_FLEET_KEY_HEX=0102030405060708090a0b0c0d0e0f10 py -3 -m pytest tests -q
```

In PowerShell: `$env:LIFETRAC_FLEET_KEY_HEX = "0102030405060708090a0b0c0d0e0f10"; py -3 -m pytest tests -q`.

That key is the dummy key CI uses. Real fleet keys (the PC's gitignored
`key.h`, the base's `secrets/`) never go in git.

## Arm GNU Toolchain 12.2.MPACBTI-Rel1 (exact)

1. Download it from developer.arm.com: *Arm GNU Toolchain Downloads*, release
   **12.2.MPACBTI-Rel1**. Take the Windows (mingw-w64-i686) host,
   AArch32 bare-metal (`arm-none-eabi`) installer,
   `arm-gnu-toolchain-12.2.mpacbti-rel1-mingw-w64-i686-arm-none-eabi.exe`.
2. Install it and tick *Add path to environment variable*. It installs to
   `C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\12.2 mpacbti-rel1\bin`.
3. Check it:

   ```
   arm-none-eabi-gcc --version
   arm-none-eabi-gcc.exe (Arm GNU Toolchain 12.2.MPACBTI-Rel1 (Build arm-12-mpacbti.34)) 12.2.1 20230214
   ```

Only this toolchain rebuilds the bench image `0c1bb0a9` and the production
image `589c1203` byte for byte. Any other GCC, including CI's Ubuntu
`gcc-arm-none-eabi`, builds working firmware with different bytes. Expected
md5s and the build steps are in
[murata_l072/README.md](../../murata_l072/README.md).

## WinLibs `mingw32-make` (PowerShell only)

```powershell
winget install BrechtSanders.WinLibs.POSIX.UCRT
```

This provides `mingw32-make.exe` (GNU Make 4.4.1 on the bench) and the host
`gcc` that `mingw32-make check` needs. If `mingw32-make` is not on PATH after
the install, it is in
`%LOCALAPPDATA%\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin`.

Run it from **PowerShell**. It fails on this bench when run from Git Bash.

## Android platform-tools (adb)

```powershell
winget install Google.PlatformTools
```

The harness finds `adb` on PATH. From Git Bash, set `MSYS_NO_PATHCONV=1` and
give local files as Windows-style paths. Otherwise MSYS rewrites `/tmp/...`
arguments into Windows paths:

```bash
export MSYS_NO_PATHCONV=1
adb -s 2E2C1209DABC240B push C:/path/to/file.py /tmp/lifetrac_strict/
```

Plug each board straight into a PC port, not a hub. Do not reach for
`adb kill-server` while boards are enumerating: it has dropped boards that
then did not come back ([ADB_TIPS_AND_TRICKS.md](../../../X8_HEALTH_AND_RECOVERY/recovery/ADB_TIPS_AND_TRICKS.md)).

## uuu 1.5.243 (OS reflash only)

You need `uuu` only to reflash an X8's eMMC over SDP
([T3a](../../../X8_HEALTH_AND_RECOVERY/recovery/T3a_sdp_uuu_reflash.md);
[BENCH_SETUP.md](BENCH_SETUP.md) pins the image). The LmP bundle ships its own
`uuu.exe` in `mfgtool-files-portenta-x8/`; NXP publishes the same version
under `github.com/nxp-imx/mfgtools/releases` (tag `uuu_1.5.243`). SDP needs
USB-C **and** 12 V on the carrier, plus a direct USB port.

## Optional

- **arduino-cli** (1.4.1 on the bench) with
  `arduino-cli core install arduino:mbed_portenta@4.5.0`. You need it only
  for the X8's H7/M4 sketches, not for radio legs. A sketch upload to the M7
  overwrites the stock x8h7 bridge ([BENCH_BOARDS.md](BENCH_BOARDS.md)).
- **SEGGER J-Link Software.** Use it only for last-resort L072 recovery over
  SWD when the ROM-bootloader flash path fails. The carrier's L072 SWD header
  CN2 is not populated by default
  ([BRINGUP_MAX_CARRIER.md](../../murata_l072/BRINGUP_MAX_CARRIER.md) §2).
  Do not accept J-Link OB firmware-update prompts.

## Firefox (camera legs)

Camera legs need moving content on the screen the camera watches. Set it up
like this:

1. Open a **normal** window of a separate profile:
   `firefox --no-remote -profile <dir>`.
2. Put a `user.js` in that profile with
   `user_pref("media.autoplay.default", 0);` and
   `user_pref("media.volume_scale", "0.0");`.
3. Open the railroad video by its watch URL.
4. Switch to the player's own fullscreen (`f`). Esc leaves it.

**Never use `firefox --kiosk`**, or anything else fully fullscreen that the
operator cannot leave: a kiosk locked the operator out of the PC for a whole
round. The window helper and the scene check are in
[legs/README.md](legs/README.md); the procedure is
[RS13_VECTOR_LEG.md](RS13_VECTOR_LEG.md), "Scene".

## Check

```powershell
git config core.longpaths          # true
py -3 --version                    # 3.11 or newer
arm-none-eabi-gcc --version        # ... 12.2.MPACBTI-Rel1 ... 12.2.1 20230214
mingw32-make --version             # GNU Make 4.4.x
adb version
```
