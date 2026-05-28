# ADB Tips, Tricks, and Lessons Learned (Portenta X8 bench)

**Scope:** Practical do/don't rules for interacting with the LifeTrac
Portenta X8 boards (tractor `2E2C1209DABC240B`, base `2D0A1209DABC240B`)
over ADB from the Windows bench host. Compiled from repeated bench
incidents 2026-05-13 through 2026-05-28.

**Companion docs:**
- [SOFT_RESET_INDEX.md](SOFT_RESET_INDEX.md) — full recovery tier catalogue
- [T0_adb_daemon_kick.md](T0_adb_daemon_kick.md) — when `kill-server` is OK
- [T2_cold_power_cycle.md](T2_cold_power_cycle.md) — when ADB is unrecoverable
- Repo memory: `lifetrac-x8-adb-reverse-broken.md`,
  `lifetrac-x8-reboot-wedges-usb.md`,
  `lifetrac-x8-passwordless-sudo.md`,
  `lifetrac-2026-05-28-post-powercycle-lora.md`

---

## 0. The golden rules

1. **Always target a serial with `-s <serial>`.** Never let ADB pick a
   "default" device when both boards may be attached.
2. **Observe before you touch.** `adb devices -l` and Windows
   `Get-PnpDevice` are read-only. Use them first to decide whether the
   board is gone at the host layer, gone at adbd, or just slow.
3. **Push a script, don't quote a script.** Multi-line remote work
   should be a `.sh` pushed to `/tmp/...` and run with
   `sudo -n sh /tmp/...`. Inline `sudo sh -c '...'` with nested quoting
   has repeatedly produced cryptic failures, half-executed pipelines,
   and `sudo` password prompts that VS Code blocks.
4. **Prefer `sudo -n` (non-interactive).** With the passwordless sudo
   drop-in installed (see [`install_lifetrac_nopasswd.sh`](../../firmware/x8_lora_bootloader_helper/install_lifetrac_nopasswd.sh)),
   never pipe a password. `sudo -n` either works or fails cleanly.
5. **One change per command while the bench is fragile.** Don't combine
   `kill-server`, `pnputil /restart-device`, and `wait-for-device` in a
   single line — when something breaks you won't know which step did it.

---

## 1. Safe commands (observed working repeatedly)

Use this exact ADB executable path so the WinGet build, not a stale
PATH copy, is used:

```powershell
$adb='C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
```

### 1.1 Pure observation (lowest risk)

```powershell
& $adb devices -l
& $adb -s 2D0A1209DABC240B get-state
& $adb -s 2E2C1209DABC240B get-state
```

`get-state` is cheaper than `shell echo` and has not been observed to
destabilize the transport.

### 1.2 Minimal shell smoke (only if listed as `device`)

```powershell
& $adb -s 2E2C1209DABC240B shell echo TRACTOR_SHELL_OK
& $adb -s 2D0A1209DABC240B shell echo BASE_SHELL_OK
```

If the shell echo causes the board to disappear from `adb devices`, that
is a board-side adbd/USB-gadget problem (see §3). Do NOT immediately
retry — stop and reassess.

### 1.3 Single-line health probe with passwordless sudo

```powershell
& $adb -s 2E2C1209DABC240B shell `
  'echo USER=$(whoami); uptime; sudo -n id; echo SUDO_RC=$?; sudo -n true; echo SUDO_TRUE_RC=$?'
```

Expected: `USER=fio`, `sudo_rc=0`, `SUDO_TRUE_RC=0`. Verified on tractor
post power-cycle (2026-05-28). Confirmed earlier on base when base ADB
was up.

### 1.4 Push-then-run (preferred pattern for any non-trivial work)

```powershell
& $adb -s 2E2C1209DABC240B push .\my_helper.sh /tmp/lifetrac/my_helper.sh
& $adb -s 2E2C1209DABC240B shell 'chmod 755 /tmp/lifetrac/my_helper.sh; sudo -n sh /tmp/lifetrac/my_helper.sh'
```

Why: avoids quoting-hell, avoids the VS Code sensitive-input guard,
keeps the remote command short enough that adbd doesn't time out.

### 1.5 Gentle wait

```powershell
& $adb -s 2D0A1209DABC240B wait-for-device
& $adb devices -l
```

`wait-for-device` is passive. Only use it scoped to a specific serial,
and always follow it with `adb devices -l` to confirm.

### 1.6 Reading container logs (preferred shape)

```powershell
& $adb -s 2E2C1209DABC240B shell 'sudo -n docker logs --tail 80 tractor-image-tx-v2 2>&1'
```

Proven good on tractor 2026-05-28. Always pin `--tail`; unbounded
`docker logs` can stall the adb shell channel.

---

## 2. Use with caution

These commands are sometimes necessary, but each has hurt us at least
once. Don't reach for them as a first response.

| Command | When it's OK | Why caution |
|---|---|---|
| `adb kill-server` / `start-server` | Only when `adb devices` is clearly lying (e.g., shows `offline` for a board that PnP shows OK). | During fragile enumeration, restarting the server has been observed to drop both boards momentarily; the base then failed to come back at all (2026-05-28). |
| `adb reconnect` / `adb reconnect offline` | If a single board is listed `offline` and you don't want to drop the other. | Has had no effect on real board-side wedges; can mask transport state. |
| `pnputil /restart-device <InstanceId>` | Last resort host-side action; needs Admin. | Will reset the USB child but does not fix the Linux-side adbd half-state, and can briefly cause both Portenta interfaces to re-enumerate. |
| `adb reboot` | If you must reboot from ADB and the board is currently healthy. | Strictly preferred over `sudo reboot` over ADB, but still: prefer a physical RESET button or power cycle while the bench is fragile. |
| `adb -s <serial> shell sudo -n docker compose ...` | When passwordless sudo is installed and the board is stable. | A blocked Docker pull or a long `compose up` can hold the shell channel and time out. Prefer a pushed script that backgrounds the work. |

---

## 3. Hard "do not" list

Each of these has been documented as actively destabilizing the bench.

### 3.1 `adb reverse` is broken on LmP 934-91

```powershell
# DO NOT
& $adb -s <serial> reverse tcp:1883 tcp:1883
& $adb -s <serial> reverse --list
```

- Observed (2026-05-24, board 2E2C): `reverse tcp:1883 tcp:1883` returns
  exit 0 but the tunnel is never installed. From the X8,
  `exec 9<>/dev/tcp/127.0.0.1/1883` returns "Connection refused".
- `reverse --list` returns `error: protocol fault (couldn't read status
  length): No error` and has been observed to drop the device from
  `adb devices` briefly. Do NOT follow it with `kill-server`.
- **Use instead:** direct LAN MQTT. From inside the X8,
  `exec 9<>/dev/tcp/192.168.1.79/1883` returned `LAN_MQTT_OK`. The image
  daemons already support `--mqtt-host` / `LIFETRAC_MQTT_HOST`.

### 3.2 `sudo reboot` over `adb exec-out` / `adb shell` can wedge USB

```powershell
# DO NOT
& $adb -s <serial> exec-out "echo fio | sudo -S reboot"
& $adb -s <serial> shell "echo fio | sudo -S reboot"
```

- Observed (2026-05-22, board 2D0A): board never re-appeared in
  `adb devices` after >10 min, multiple `kill-server` cycles, and
  `reconnect offline`. Required physical USB reseat / power cycle.
- Fingerprint (see `lifetrac-x8-reboot-wedges-usb.md`): Windows
  `Get-PnpDevice` still shows the X8 USB Composite + ADB Interface +
  COM Serial child all `Status=OK`, but adb sees nothing and the
  Linux-side adbd + getty have stopped draining their FunctionFS
  endpoints. The COM child opens cleanly but the first `WriteLine`
  raises `IOException: semaphore timeout`.
- **Use instead:** `adb reboot` (if a soft reboot is truly needed) or
  the physical RESET button on the Max Carrier. For an HW reset of the
  i.MX path, see [T2_cold_power_cycle.md](T2_cold_power_cycle.md).

### 3.3 Don't pipe board passwords through `sudo -S`

- VS Code's sensitive-input guard now intercepts the `[sudo] password
  for fio:` stderr line even when piping via `echo fio | sudo -S ...`.
  Commands hang or get cancelled mid-flight.
- `sudo -p ''` suppresses the prompt label but is fragile; the password
  itself is still in command history and tool calls.
- **Use instead:** install the passwordless drop-in once per board,
  then use `sudo -n ...` everywhere. See
  [`install_lifetrac_nopasswd.sh`](../../firmware/x8_lora_bootloader_helper/install_lifetrac_nopasswd.sh).

### 3.4 Don't loop-retry after a board disappears

If a board drops from `adb devices`:

- **Do not** immediately rerun the same command.
- **Do not** `kill-server; start-server; reverse; reconnect` in a row.
- **Do** stop, observe `adb devices -l` + `Get-PnpDevice`, and decide
  whether the loss is at the host layer, at adbd, or at USB. Repeated
  retries have several times converted a recoverable adbd hiccup into a
  USB-gadget half-up state that only a physical power cycle fixes.

### 3.5 Don't nest heavy quoting in `adb shell`

PowerShell → cmd-style `adb shell` → remote `sh -c '...'` is a
quoting-escape minefield. Failures look like silent partial execution.
**Use push-then-run (§1.4).**

---

## 4. Diagnostic recipes

### 4.1 "Where did the board go?" triage

```powershell
$adb='C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe'
Write-Output '--- adb ---'
& $adb devices -l
Write-Output '--- Windows PnP (Portenta VID 2341) ---'
Get-PnpDevice -PresentOnly |
  Where-Object { $_.InstanceId -match 'VID_2341' -or
                 $_.FriendlyName -match 'ADB|Portenta|Arduino|USB Serial|Composite' } |
  Select-Object Status, Class, FriendlyName, InstanceId | Format-List
```

Interpretation:

| `adb devices` | PnP shows board | Diagnosis | Next step |
|---|---|---|---|
| Listed `device` | Yes | Healthy | Proceed |
| Listed `offline` | Yes | Stale host transport | `adb kill-server` / `start-server` is acceptable here |
| Missing | Yes (Composite + ADB Interface) | Board-side adbd / FunctionFS hung | **Do not** kill-server first; try `wait-for-device <serial>` once, else physical RESET |
| Missing | No (only one X8 present) | Board is physically gone (cable, power, reset) | Power cycle / cable reseat |

### 4.1.1 Unknown USB Device / descriptor failure

If Device Manager shows:

```text
Unknown USB Device (Device Descriptor Request Failed)
USB\VID_0000&PID_0002\...
HardwareIds: USB\DEVICE_DESCRIPTOR_FAILURE
ProblemCode: 43
```

Windows did not get far enough to read the Portenta descriptor, serial,
interfaces, or ADB function. This is **below ADB** and below the Arduino
VID/PID driver binding layer.

Observed 2026-05-28 after a Windows reboot while recovering base
`2D0A1209DABC240B`:

- Tractor `2E2C1209DABC240B` enumerated normally as
  `USB\VID_2341&PID_0061\2E2C1209DABC240B` plus COM + ADB children.
- Base appeared only as `Unknown USB Device (Device Descriptor Request
  Failed)` / `USB\VID_0000&PID_0002` / ProblemCode 43.
- Elevated `pnputil /restart-device` on the Unknown device succeeded
  but did **not** recover ADB.
- Elevated `pnputil /scan-devices` completed and the same Unknown device
  reappeared.

Verdict: host-side PnP can re-trigger enumeration, but if the device
keeps returning descriptor failure, the likely next steps are physical
USB cable reseat, parent hub power/reset, or base-board reset/power
cycle. Do not spend time on `adb kill-server`, drivers, `adb reconnect`,
or `adb reverse`; ADB cannot talk to a device that never supplied a USB
descriptor.

**Recovery sequence that actually worked (2026-05-28):**

1. Two full Windows host reboots — no effect.
2. Elevated `pnputil /restart-device` on the Unknown device, on its
   parent USB hub, and `pnputil /scan-devices` — no effect.
3. Disabled USB selective suspend on the parent hub
   (`EnhancedPowerManagementEnabled=0`, `SelectiveSuspendEnabled=0`,
   `AllowIdleIrpInD3=0` under the hub's `Device Parameters`) — no
   effect, but harmless to leave in place.
4. Physical USB cable reseat at both ends — no effect.
5. **Cold power cycle of the base board** — fully recovered. Base
   re-enumerated as `USB\VID_2341&PID_0061\2D0A1209DABC240B` with
   COM12 + ADB Interface MI_02 children, and `adb devices` listed it
   as `device` on the next poll.

Operator observation at the moment of recovery: the X8 user LED came
up **yellow** (not the usual healthy color) after the power cycle. Note
this as a recovery-state indicator; if you see yellow LED + good ADB,
the board is reachable but may still be in a partially-initialized
userland state — confirm with §4.2 before driving any LoRa/container
work.

Diagnostic side-finding: when both boards share a Generic USB Hub
(here `USB\VID_05E3&PID_0610`), querying
`DEVPKEY_Device_Children` on the hub will list both boards and let
you prove the hub itself is fine when the other board enumerates
normally on it. That isolates the fault to the specific port, the
cable, or the failing board's USB-C connector / transceiver — not
the host stack.

### 4.2 Minimum stable-board confirmation

```powershell
& $adb -s <serial> get-state
& $adb -s <serial> shell echo SHELL_OK
& $adb -s <serial> shell 'sudo -n true; echo SUDO_RC=$?'
& $adb devices -l
```

If all four succeed in a row, the board is ready for real work (push
scripts, start daemons, read logs).

### 4.3 LoRa-side smoke after ADB is confirmed

Once §4.2 passes on a board:

- Tractor: read `tractor-image-tx-v2` logs via §1.6, look for
  `CFG_SET_REQ(REG_PROFILE=0) OK`, `FRF readback ... 915.000 MHz (OK)`,
  and recurring `RFCO_PERTX ... tx_status=0x00(OK)`.
- Base: bring up the RX stack via a pushed helper, then read
  `image_rx_daemon` / `lora_bridge` logs the same way.

Do NOT start tractor TX before base RX is confirmed up — transmitting
into a missing receiver wastes airtime and obscures pipeline state.

---

## 5. One-page cheat sheet

```text
SAFE:
  adb devices -l
  adb -s <serial> get-state
  adb -s <serial> shell echo OK
  adb -s <serial> shell 'sudo -n id; echo SUDO_RC=$?'
  adb -s <serial> push <file> /tmp/...
  adb -s <serial> shell 'chmod 755 /tmp/x.sh; sudo -n sh /tmp/x.sh'
  adb -s <serial> shell 'sudo -n docker logs --tail 80 <ctr> 2>&1'
  adb -s <serial> wait-for-device

CAUTION (single-purpose, deliberate):
  adb kill-server / start-server
  adb reconnect
  adb reboot
  pnputil /restart-device <InstanceId>   (Admin)

DO NOT:
  adb -s <serial> reverse tcp:1883 tcp:1883
  adb -s <serial> reverse --list
  adb -s <serial> exec-out "echo fio | sudo -S reboot"
  adb -s <serial> shell  "echo fio | sudo -S reboot"
  adb -s <serial> shell 'sudo sh -c "<deeply nested quoted pipeline>"'
  Loop-retry the same command after the board disappears.
```

---

## 6. Change log

- 2026-05-28: Initial version. Sources: repo memory files listed in
  header, transcript evidence from 2026-05-13, 2026-05-22, 2026-05-24,
  and 2026-05-27/28 bench sessions.
- 2026-05-28 (later): Expanded §4.1.1 with the actual recovery
  sequence for base `2D0A1209DABC240B` Code 43. Confirmed that two
  Windows reboots, all `pnputil` device-restart variants, USB
  selective-suspend disable, and a physical cable reseat were
  ineffective; **cold power cycle of the board** was the only thing
  that cleared the descriptor failure. Recorded yellow X8 LED as a
  post-recovery observation and added the
  `DEVPKEY_Device_Children` shared-hub isolation technique.
