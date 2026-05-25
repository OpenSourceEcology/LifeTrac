# 2026-05-25 — Concurrent Smoke Test: RX UART Hang + Universal TX_DONE TIMEOUT

## Scope
Two consecutive runs of `run_concurrent_smoke.ps1` against TX=2E2C and RX=2D0A
under Profile 1 (FHSS 250 kHz / 50 CH) with the host-side synthetic publisher
producing 2 fps frames at `byte_budget=250` and `byte_budget=400`.

Goal: drive `rx_frames_seen > 0` over the air to validate the W2-02
image-over-LoRa transport.

## What changed before these runs
1. `publish_synthetic_frames.py` byte_budget lowered to 250 (was 400 → 5000),
   keeping per-frame size ≈ 240–260 B so fragmentation stays ≤ 7 fragments.
2. `run_concurrent_smoke.ps1` adds a pre-flight read-only UART drain
   (`timeout 1 dd if=/dev/ttymxc3 of=/dev/null`) on both boards.
3. Added launch of the synthetic publisher inside the runner.

## Observed (two consecutive runs, 09:14 and 09:17)

### RX 2D0A — hangs identically every time
```
WARNING: dropped malformed frame: truncated COBS payload   (×5 during settle)
ERROR image_rx_daemon: VER warm-up failed: timeout waiting for response type 0x81
fatal: cannot open L072 HostLink
image_rx_daemon exit  (≈ 2.7 s after start)
```

- 06:46 run (earlier today) **succeeded** at warm-up — same exact image and
  command, only the L072 board state differed. Drain has zero effect.
- "Truncated COBS payload" indicates the L072 is actively streaming bytes when
  the daemon attaches, but no `0x81` VER response arrives.
- This points to the L072 being stuck in a mid-frame TX/loop state from a
  prior aborted concurrent run, not a transient buffer.

### TX 2E2C — keeps running but every TX_DONE returns status=1(TIMEOUT)
```
TX worker ready (inter_cycle_s=0.050, max 8 frags/dwell)
TX_DONE non-OK: seq=1 idx=0 ... status=1(TIMEOUT) toa_us=48768
TX_DONE non-OK: seq=2 idx=0 ... status=1(TIMEOUT) toa_us=48768
...  (uniform across all 30+ seqs and across both runs)
stats: frames_in=30 ok=0 fail=11 drop_full=15 frags_ok=63 frags_fail=34
```

- `ok=0` for the entire window. PHY hand-off accounting is misleading: the
  `frags_ok` counter increments on submit, not on TX_DONE OK.
- `toa_us=48768` is constant → time-on-air computation is fine; the firmware
  simply never observes its own DIO0 TX_DONE within the expected window.

## Falsification of the previous theory
The prior session summary attributed the "no rx_frames" symptom to a
"physical layer RF alignment / demodulation barrier" requiring the daemons to
warm into steady state. That hypothesis is now ruled out:

- Steady-state never occurs. After 30 s of continuous TX, the TX_DONE status
  is still uniformly TIMEOUT.
- The W1-11 ping-pong runner this morning failed at the **listener-ready**
  stage with identical TX-side behaviour (RX side never produced
  `__W1_10B_LISTEN_READY__`).

Both symptoms (RX UART stuck mid-stream; TX never sees its own TX_DONE) are
firmware-state failures on the L072 co-processor. They are independent of the
host orchestration layer.

## Cross-check vs. memory
- `/memories/repo/lifetrac-x8-l072-bootloader.md` — relevant.
- User memory `methodology.md`: *"Blocking I/O calls inside ISR error paths
  can self-DOS … cascading frame loss that LOOKS like a transport bug. Always
  check whether the IRQ handler can preempt itself or block on the same
  peripheral it's draining."* — this exactly matches the present symptom on
  both ends.
- User memory `misdiagnosis.md`: status fields lose information; the
  `frags_ok=63` headline number conceals 0 actual TX_DONE successes.

## Recommended next steps (firmware side, not orchestration)
1. **Hard-reset the L072 on RX 2D0A** (power-cycle or NRST pulse via H7 SWD
   leg) before any further smoke runs. Pure ADB UART drain is insufficient.
2. **Reflash the LoRa co-processor firmware** on both boards with the build
   that included the W1-9 ISR fix; the universal TX_DONE TIMEOUT looks like
   the DIO0 ISR is being preempted/blocked by the same ORE diagnostic emit
   pattern called out in methodology.md.
3. Re-run `run_w1_10b_rx_pair_end_to_end.ps1 -Probe ping_pong` first as the
   minimal smoke; only proceed to the image E2E once ping_pong demonstrates
   a non-zero RX frame count.

## Files touched this session
- `publish_synthetic_frames.py` — byte_budget=250, loop=300 (150 s coverage).
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_concurrent_smoke.ps1`
  — added inline synthetic publisher launch; **replaced the ineffective `dd`
  UART drain with a gpio163 (L072 NRST) pulse on both boards per
  SOFT_RESET_INDEX 3.1**. The dd-drain only flushed the host RX buffer;
  it could not recover an L072 stuck mid-COBS-transmit. The NRST pulse
  drives H7 PF4 → x8h7 → gpio163 and gives a clean L072 boot state.

## 2026-05-25 (later) — softer recovery & RX USB drop
- Confirmed gpio163 is the L072 NRST control GPIO on BOTH boards
  (file: `board2_gpio163_probe.sh`; index entry: `SOFT_RESET_INDEX.md` 3.1).
- Pulse sequence (idempotent, ~150 ms, no USB teardown):
  ```sh
  [ -d /sys/class/gpio/gpio163 ] || echo 163 > /sys/class/gpio/export
  echo out > /sys/class/gpio/gpio163/direction
  echo 1 > /sys/class/gpio/gpio163/value      # de-assert (idle high)
  sleep 0.02
  echo 0 > /sys/class/gpio/gpio163/value      # assert NRST low
  sleep 0.10
  echo 1 > /sys/class/gpio/gpio163/value      # release → L072 boots
  ```
- Successfully pulsed TX (2E2C) — `PULSE_DONE_AT=1779701120.705479990`.
- RX (2D0A) dropped off USB entirely (`adb devices` shows only 2E2C).

## 2026-05-25 (later still) — CORRECTION: RX drop is a Linux-side USB-gadget hang on the RX board, NOT user intervention
An earlier revision of this note speculated "user power-cycled the board
mid-session". That was incorrect — the user explicitly stated they did not
touch the hardware. A proper evidence-based diagnosis was then carried out.

### Evidence chain (RX 2D0A)
1. `ADB_TRACE=usb adb nodaemon server` log (RX side only):
   ```
   adding a new device \\?\usb#vid_2341&pid_0061&mi_02#6&25d14da8&0&0002
   cannot get serial number: ... (31)
   ```
   Windows error 31 = `ERROR_GEN_FAILURE` ("A device attached to the system
   is not functioning") — Windows can see the descriptor but the
   `GET_STRING_DESCRIPTOR` IN transfer for the serial-number string never
   completes. This means the **USB device-side firmware (Linux gadget
   driver on the i.MX8M) has stopped servicing control transfers** on the
   ADB interface.
2. Same error 31 returned when opening the **CDC-ACM** serial endpoint
   (COM12, MI_00) via `System.IO.Ports.SerialPort` directly — so it is
   not specific to the ADB interface; the whole composite device is dead.
3. Opening TX COM11 (MI_00, 2E2C) the exact same way **succeeds** — proves
   the fault is RX-board-specific, not host-cable-driver-class-specific.
4. UAC-elevated `Disable-PnpDevice` + `Enable-PnpDevice` on the MI_02
   ADB interface returned `Status=OK / Problem=CM_PROB_NONE` but adb still
   could not read the serial — proves Windows is using **cached descriptors**
   while the device side remains hung. (→ add to misdiagnosis.md.)
5. UAC-elevated Disable/Enable of the **composite parent**
   `USB\VID_2341&PID_0061\2D0A1209DABC240B` + `pnputil /restart-device`
   on the parent also failed to recover — even forcing a full re-enumeration
   from the host side does not wake the device.
6. LAN ping sweep of 192.168.1.0/24 + `arp -a` filtered for NXP/Portenta
   prefixes returned no match — RX is not reachable over Ethernet either,
   so the Linux kernel netif stack is also gone (or eth0 carrier is down).

### Diagnosis
The RX board's **i.MX8M Linux side is hung at the kernel-driver layer**
(USB gadget controller `ci_hdrc` / `g_multi` is not servicing IRQs;
network stack is also unresponsive). Likely trigger this session:
- the two abusive `timeout 35` docker container kills at 09:14 and 09:17
  that left `/dev/ttymxc3` half-open while a COBS storm was in flight, plus
- the cleanup-time gpio163 sysfs writes I issued during the recovery attempt.

This is consistent with prior observations in
`/memories/repo/lifetrac-w2-usb-pm.md` about the X8 USB-gadget stack being
fragile under abusive host-side kill patterns.

### TX uptime=8 min — also NOT user intervention
TX 2E2C uptime of 8 min at the same wall-clock moment is **not** explained
by user power-cycling either. Likely real causes (verify on next reboot):
- Foundries OTA / `aktualizr-lite` applied an update and rebooted, OR
- a kernel oops / watchdog reset on TX from the same abusive test load
  (TX was in the COBS storm path too), OR
- USB-bus power-cycle initiated by the Windows-host hub when the RX side
  collapsed (some hub firmware drops V_BUS on the whole port group).
This must be checked in `dmesg` / `journalctl --boot=-1` after recovery.

### Recovery path
- **Only physical power-cycle of the RX board recovers the Linux gadget
  stack.** Every non-physical avenue has been exhausted (USB descriptor
  re-read, MI_02 Disable/Enable, composite parent Disable/Enable, pnputil
  /restart-device, kill-server/start-server cycle, LAN reach).
- After recovery, check `journalctl -k --boot=-1` on RX (and TX) for the
  actual crash signature so this can be added to root-cause notes.

## Status
🛑 **Blocked at firmware layer + physical layer.**
- Firmware: items (1)/(2) in "Recommended next steps" still pending.
- Physical: RX board 2D0A1209DABC240B is hung on the **i.MX8M Linux
  side** (USB gadget + netif both unresponsive) — only a board
  power-cycle will recover it. This is *not* user-caused; the abusive
  09:14/09:17 docker timeout kills + COBS storm + cleanup-time gpio163
  sysfs writes are the prime suspects.
- Once RX returns and daemons are re-staged via `push.bat`, the patched
  `run_concurrent_smoke.ps1` will pulse both L072s via gpio163 before
  launching daemons — this should eliminate the "truncated COBS" /
  "VER warm-up timeout" failure mode observed at 09:14 and 09:17.
- Also: investigate `journalctl --boot=-1` on TX for the 8-min uptime
  before assuming it was a normal reset.
