# 2026-05-14 — USB Wedge Software Mitigation Research

**Author:** GitHub Copilot
**Scope:** Software-side mitigations for the Portenta X8 / Max Carrier USB-host wedge seen when the Kurokesu C2 UVC camera is attached or opened. This is about reducing `ci_hdrc-imx` driver failure risk from camera power and resume timing; it is not about the unrelated STM32L072 option-byte brick.

## 1. Current Diagnosis

The camera did not damage the board. The lasting Board 2 failure was caused by the L072 option-byte write-unprotect path, not by the Kurokesu C2.

The camera can still trigger a separate X8 Linux failure mode. The C2 idles around the low hundreds of milliamps, can step much higher when the imager and USB streaming path are enabled, and appears as a composite UVC/UAC device behind the Max Carrier's SMSC USB 2.0 hub. On this hardware stack, that power and enumeration transient can combine with USB runtime PM and the `ci_hdrc-imx` host driver into a hard host-controller wedge. Symptoms include `error -71`, USB descriptor failures, lost ADB/userland service, and recovery only after reset or full power cycling.

The upstream Linux USB PM docs sharpen the key point: writing `power/control=on` to an already-suspended USB device is not harmless. It requests a resume and disables future autosuspend. If the C2 has already runtime-suspended, that write enters the same dangerous autoresume path as a later V4L2 `open()`.

## 2. Research Findings

### 2.1. Camera Boot Power Is Probably Not Software-Configurable

The Kurokesu C2 behaves as a standard USB UVC camera. Linux exposes standard V4L2/UVC controls and, in principle, vendor extension-unit controls can exist, but those controls are only reachable after USB enumeration and driver binding. They cannot reduce the initial VBUS inrush or the first sensor/ISP boot cycle.

I found no reliable evidence that the C2 exposes a persistent "boot in low power" setting. Treat the camera's USB attach behavior as fixed. Software can reduce risk after enumeration; it cannot remove the electrical step at VBUS attach.

### 2.2. `usbcore.autosuspend=-1` Is the Highest-Value Software Mitigation

Kernel documentation says `usbcore.autosuspend=<seconds>` controls the default autosuspend delay for newly detected USB devices. A negative value prevents autosuspend. The default is normally `2`, which means a device can suspend after two idle seconds.

This matches our failure pattern: the orchestrator can leave the C2 idle while files are pushed and commands are staged, then the next sysfs write or V4L2 open triggers autoresume/reset on a fragile host controller.

Important implementation detail: changing `/sys/module/usbcore/parameters/autosuspend` at runtime only affects USB devices detected after the write. It does not retroactively fix an already-enumerated C2.

### 2.3. Late Per-Device `power/control` Writes Are Racy

`power/control=on` and `power/autosuspend_delay_ms=-1` are useful only when applied before the C2 has suspended, or when the device is known to still be `active`. The current helper's cautious gate is correct: check global autosuspend and `runtime_status`, start `dmesg -wH` first, and skip a write if the target is already `suspended`.

The best implementation is to prevent C2 suspend before it ever happens, not to repair it afterward.

### 2.4. `snd-usb-audio` Should Be Disabled for the C2

The C2 exposes a microphone/audio interface. The `cannot get freq at ep 0x84` message we saw is from `snd-usb-audio`, not `uvcvideo`, and is usually benign. Even so, the audio driver can still allocate periodic USB bandwidth and perform alternate-setting negotiation on the same composite device before video starts.

Because LifeTrac does not need the C2 microphone, the right policy is to leave the video interface bound to `uvcvideo` and unbind or blacklist the C2 audio interface. This reduces USB isochronous scheduling pressure and removes one more driver from the risky initialization sequence.

### 2.5. UVC Quirks Are Fallbacks, Not Primary Fixes

`UVC_QUIRK_FIX_BANDWIDTH` (`quirks=0x80`, decimal `128`) helps when a camera overclaims bandwidth for uncompressed streams and the host returns `-ENOSPC`. It is much less compelling for our C2 path because the preferred capture mode is MJPEG, the upstream FAQ says compressed streams are harder for the driver to estimate, and our failure is a host wedge rather than a clean `-ENOSPC` return.

Use UVC quirks only after autosuspend and audio-interface policy are in place. Do not depend on `uvcvideo.nodrop=1`; upstream is removing that parameter.

### 2.6. Smaller Capture Modes Help Only After Enumeration

Starting with MJPEG and a modest resolution/fps can reduce the current jump at `STREAMON`, CPU load, and USB bandwidth pressure. It does not solve the initial VBUS attach event. A good first capture target is enumerate-only, then a small MJPEG or YUYV stream, then the desired 1280x720 MJPEG capture.

## 3. Implementation Recommendation

My recommendation is to split the mitigation into a root-owned USB policy step and an unprivileged camera capture step.

The current helper bundles some USB setup behind `--UseSudo`. That is risky as a long-term shape because the safer permission model is to add `fio` to the `video` group and run capture without sudo. If capture becomes unprivileged but USB mitigation remains tied to `--UseSudo`, the safest capture path silently skips the USB defenses.

### 3.0. Voltage-Drop-Specific Techniques

Software cannot raise VBUS, but it can change *when* current is drawn and *how concurrent* the load is. The wedge fires hardest when VBUS attach, hub-port enable, USB enumeration, `uvcvideo` probe, `snd-usb-audio` probe, and any sensor self-cal all stack on the same few hundred milliseconds. Every technique below decouples one of those events from the others so the rail has time to recover.

**a. Hold the device unauthorized until the system is quiescent.** This is the strongest software lever for voltage drop because it lets enumeration finish (a small, mostly control-transfer load) but blocks driver binding (which is when isochronous bandwidth is reserved and the sensor/ISP is fully woken).

```bash
# Boot policy: new USB devices come up unauthorized.
echo 0 > /sys/bus/usb/devices/usb1/authorized_default

# Later, after dmesg is quiet and no other USB activity is in flight:
echo 1 > /sys/bus/usb/devices/1-1.2/authorized   # path of the C2
```

The kernel parameter `usbcore.authorized_default=0` makes this the default at boot. Combined with `usbcore.autosuspend=-1`, the C2 enumerates with no driver attached, sits idle drawing only its quiescent current, and only takes the larger step when we explicitly authorize it.

**b. Use port-power gating on the powered hub when it arrives.** A USB 2.0 hub that exposes per-port power control (`uhubctl`-compatible) lets us turn the C2's VBUS off and on from software. That is the cleanest way to guarantee the camera is fully off during X8 boot and only powers up after `usbcore.autosuspend=-1` and unauthorized-default are in effect. It also gives us a software-recoverable wedge path: if capture fails, cycle the port instead of cycling the X8.

**c. Stagger driver binding.** Even after authorization, do not let `uvcvideo` and `snd-usb-audio` race. The simplest implementation is to blacklist `snd_usb_audio` system-wide (§3.2). If we ever need USB audio later, the alternative is to keep the audio driver unloaded until after the first successful video `STREAMON`, then `modprobe snd_usb_audio` if needed.

**d. Quiet other USB activity during camera attach.** Suspend or unbind anything else on the same controller during the bring-up window: USB-Ethernet, USB-serial bridges, mass storage probing, `bluetooth` (if USB-attached). Less concurrent enumeration means smaller superimposed current steps on the same rail.

**e. Start small at STREAMON.** First successful capture should be the lowest MJPEG mode the camera advertises (often 320x240 or 640x480 at 15 fps). The sensor and ISP draw less, the isochronous bandwidth reservation is smaller, and the X8 has more margin if anything else twitches. Step up to 1280x720 only after a known-good small capture.

**f. Add a settle delay between authorize and open.** After authorizing the C2 (or after `udev` reports it `bound`), wait 500–1000 ms before the first V4L2 `open()`. Cheap and removes a class of races where the sensor is still in its boot self-cal when we ask it to stream.

**g. Do not pulse `power/control`.** Some online guides suggest writing `auto` then `on` to "wake" a stuck device. On this hardware that is exactly the autoresume path that wedges us. Never toggle; set once, early, and leave it.

The order matters: a + e + f are the three with the highest leverage that we can implement today without new hardware. b is the highest-leverage hardware-assisted technique once the powered hub arrives.

### 3.1. Bench-Safe Sequence

1. Boot the X8 with the C2 unplugged, or with the C2 behind a controllable/powered hub that can keep camera VBUS off until Linux is ready.
2. As root, before plugging or powering the C2, run:

```bash
echo -1 > /sys/module/usbcore/parameters/autosuspend
echo 0  > /sys/bus/usb/devices/usb1/authorized_default   # adjust per controller
cat /sys/module/usbcore/parameters/autosuspend
```

3. Start `dmesg -wH` logging before any sysfs walk, V4L2 open, or STREAMON.
4. Plug or power the C2. Confirm in `dmesg` that the device enumerated but no driver bound (`uvcvideo` and `snd-usb-audio` should be silent).
5. Wait at least 500 ms, then authorize:

```bash
C2=$(grep -l 16d0 /sys/bus/usb/devices/*/idVendor | xargs -n1 dirname | \
     xargs -I{} sh -c 'grep -l 0ed4 {}/idProduct >/dev/null && echo {}')
echo 1 > "$C2/authorized"
udevadm settle --timeout=3
```

6. Run enumerate-only first. If that survives, run the smallest MJPEG mode the camera advertises. Then step up to the target mode.

This sequence is the best software-only bench stopgap because the C2 enumerates with no driver attached, draws only its quiescent current, and only takes the larger step on our schedule.

### 3.2. Production-Clean Path

For production, set `usbcore.autosuspend=-1` through the Foundries factory kernel-argument mechanism (`OSTREE_KERNEL_ARGS` for the `lmp-xwayland` image). Local `fw_setenv bootargs` and direct BLS `options` edits were already tested on this LmP build and did not affect `/proc/cmdline`, so they should not be used as the production path.

Then add a C2-specific udev rule as belt-and-braces:

```udev
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", \
	ATTR{idVendor}=="16d0", ATTR{idProduct}=="0ed4", \
	TEST=="power/control", ATTR{power/control}="on", \
	TEST=="power/autosuspend_delay_ms", ATTR{power/autosuspend_delay_ms}="-1"
```

For audio, prefer a targeted interface unbind rule if we want to preserve other USB audio devices. If LifeTrac never needs USB audio on the X8, the simpler robust answer is:

```conf
# /etc/modprobe.d/lifetrac-no-usb-audio.conf
blacklist snd_usb_audio
```

### 3.3. Script Changes I Would Make Next

1. Create a small root preflight script, for example `w2_01_camera_usb_guard.sh`, that only handles USB PM/audio policy and logging. It should be safe to run before capture and should not depend on `--UseSudo` capture mode.
2. Keep `w2_01_camera_first_light.sh` focused on enumeration/capture. It can verify the guard state (`autosuspend=-1`, C2 `power/control=on`, `snd-usb-audio` absent/unbound), but it should not be the only place the guard can run.
3. Update the PowerShell orchestrator to run the guard first when requested, pull the guard/dmesg logs separately, then run capture as `fio` once `fio` is in the `video` group.
4. Leave the current safety gate in place: never write `power/control=on` to a C2 whose `runtime_status` is already `suspended`.
5. Make the default first-light mode enumerate-only. Require an explicit flag for STREAMON/full capture until the powered hub or factory kernel arg is validated.

### 3.4. Optional Systemd Stopgap

A oneshot systemd service can write the runtime default at boot:

```ini
[Unit]
Description=LifeTrac USB autosuspend guard for camera bring-up
After=sysinit.target
Before=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'echo -1 > /sys/module/usbcore/parameters/autosuspend'

[Install]
WantedBy=multi-user.target
```

This is useful only if the C2 is not enumerated until after the service runs. If the camera is already attached and powered during kernel boot, use the factory kernel argument instead.

## 4. Validation Plan

1. With C2 unplugged, confirm `cat /sys/module/usbcore/parameters/autosuspend` reports `-1` and `authorized_default` reports `0` on the target controller.
2. Plug C2. Confirm `dmesg` shows enumeration but **no** `uvcvideo` or `snd-usb-audio` bind lines, and `lsusb -t` lists the device with `Driver=` blank.
3. Authorize the C2 and confirm `power/control=on`, `power/autosuspend_delay_ms=-1` if that file exists, and `snd-usb-audio` is not bound to any C2 interface.
4. Run enumerate-only and save `lsusb -t`, `/dev/v4l/by-id`, and `dmesg -wH` logs.
5. Run the smallest advertised MJPEG mode (e.g. 640x480 @ 15 fps) and verify ADB remains alive after the command returns.
6. Only then run the desired 1280x720 MJPEG capture.
7. Record one full bench cycle (boot → authorize → small capture → target capture → idle → re-capture) without a wedge before declaring the mitigation effective.

## 5. Bottom Line

Best software path: prevent runtime suspend before the C2 enumerates, disable the unused audio interface, capture kernel logs before touching USB sysfs, and keep capture unprivileged. That will not remove the camera's voltage transient, but it removes the most likely Linux-side wedge trigger.

Best overall path: use a powered USB hub in parallel. Software mitigation can make the X8 less fragile, but a powered hub directly removes the VBUS sag/inrush stress that software cannot control.

## 6. References

- Project root-cause note: [2026-05-12_W2-01_USB_Wedge_Root_Cause_Research_v1_0.md](2026-05-12_W2-01_USB_Wedge_Root_Cause_Research_v1_0.md)
- Project avoidance strategy: [2026-05-12_W2-01_USB_Wedge_Avoidance_Research_Copilot_v1_0.md](2026-05-12_W2-01_USB_Wedge_Avoidance_Research_Copilot_v1_0.md)
- Linux USB power management: https://docs.kernel.org/driver-api/usb/power-management.html
- Linux kernel parameters (`usbcore.autosuspend`): https://www.kernel.org/doc/html/latest/admin-guide/kernel-parameters.html
- Linux UVC FAQ: https://www.ideasonboard.org/uvc/faq/
- Linux UVC driver XU controls: https://www.kernel.org/doc/html/latest/userspace-api/media/drivers/uvcvideo.html

## 7. Coverage for Other USB Peripherals (GPS, IMU, Coral)

The W2-02 / W2-03 plan adds GPS and IMU to the X8 USB bus. Per
[BUILD-CONTROLLER/01_bill_of_materials.md](../BUILD-CONTROLLER/01_bill_of_materials.md)
Tier 1 and [DESIGN-CONTROLLER/HARDWARE_BOM.md](../DESIGN-CONTROLLER/HARDWARE_BOM.md), the canonical sensor stack is:

- **Adafruit MCP2221A** (Adafruit 4471) — single USB HID device (`04d8:00dd`),
  driver `hid-mcp2221`. Connects to the X8 USB bus via one USB-A port.
- **SparkFun NEO-M9N Qwiic GPS** (GPS-15733) — I²C device on the MCP2221A's
  Qwiic bus. Not visible to USB at all.
- **SparkFun BNO086 Qwiic IMU** (DEV-22857) — I²C device on the same Qwiic
  daisy-chain. Also not visible to USB.

This is the key fact: from the X8's USB-host perspective, the GPS and IMU
are NOT separate USB devices. They sit behind the MCP2221A as an I²C
daisy-chain. The X8 sees one HID device.

**Do they need the same protections as the C2? No, and here is why.**

| Risk factor          | Kurokesu C2                        | MCP2221A + GPS + IMU                           |
| -------------------- | ---------------------------------- | ---------------------------------------------- |
| USB devices on host  | 1 composite (UVC + UAC)            | 1 HID                                          |
| Driver count at bind | `uvcvideo` + `snd-usb-audio`       | `hid-mcp2221` only                             |
| Isochronous endpoint | Yes (UVC + UAC)                    | No                                             |
| Inrush at attach     | High (sensor + ISP boot, ~500 mA+) | Negligible (~10–20 mA HID; ~50–80 mA total w/ Qwiic loads) |
| Snd-usb-audio path   | Yes (`cannot get freq at ep 0x84`) | None                                           |
| Bandwidth pressure   | UVC alt-setting reservation        | None (HID interrupt only)                      |
| Wedge history        | Confirmed                          | None observed                                  |

The MCP2221A draws on the order of 10–20 mA over USB; the downstream NEO-M9N
adds ~30 mA when actively acquiring a fix; the BNO086 adds ~12 mA when its
fusion engine is running. All of that is sourced from the MCP2221A's 3.3 V
LDO, not directly from VBUS. Total VBUS current is in the **50–80 mA range**,
which is a rounding error compared to the C2's hundreds of milliamps.

**Recommendation: do not run the W2-01 USB guard for the MCP2221A.**

- `usbcore.autosuspend=-1` (the global setting that the guard or the
  factory kernel arg installs) already covers every future USB device on
  the host, so the MCP2221A inherits the autosuspend mitigation for free.
- The MCP2221A has no UAC interface, so the `snd-usb-audio` blacklist is
  a no-op for it.
- The authorization gate is unnecessary because the device's attach
  current step is too small to matter.
- Per-device `power/control=on` is also unnecessary; if you want a
  belt-and-braces guarantee, add a one-line udev rule keyed on the
  MCP2221A's VID:PID `04d8:00dd`. Cost is essentially zero.

The one thing the MCP2221A genuinely benefits from is the **production
kernel arg** `usbcore.autosuspend=-1`, because Adafruit's Blinka library
opens the HID device only when a Python service starts; if the device
suspended in the meantime, the open path is again the autoresume path that
wedged us with the C2. The probability of a wedge here is much lower because
the HID re-bind is cheaper than UVC, but the same defense is free for the
MCP2221A once the kernel arg is in place.

**Suggested udev rule for the MCP2221A** (optional belt-and-braces, file
`99-lifetrac-mcp2221a.rules`):

```udev
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", \
    ATTR{idVendor}=="04d8", ATTR{idProduct}=="00dd", \
    TEST=="power/control", ATTR{power/control}="on", \
    TEST=="power/autosuspend_delay_ms", ATTR{power/autosuspend_delay_ms}="-1"
```

### 7.1. Future USB peripherals to revisit

A few items in the BOM and TODO are not on the tractor X8 today but will
become USB peripherals later. Each is worth re-checking against the same
risk table:

- **USB galvanic isolator (ADuM3160)** — listed in
  [DESIGN-CONTROLLER/TODO.md](../DESIGN-CONTROLLER/TODO.md) as production-build
  hardening between the X8 and the MCP2221A. The isolator is its own USB
  hub from the host's perspective and adds another `1-1.x` node in `lsusb -t`.
  Not a wedge risk, but the udev rule above should be re-checked because
  the C2/MCP2221A bus addresses will shift after insertion.
- **Coral USB Accelerator** (Tier 2, base-station X8 only) — this is the
  *one* peripheral that genuinely shares the C2's risk profile. The Coral
  is famous for high inrush at first inference (multi-amp transient) and
  has wedged USB controllers on under-powered hosts. It lives on the base
  station, not the tractor, so it is on a different X8 from the camera.
  If we ever co-locate both on one host, treat the Coral exactly like the
  C2: hold authorization until the system is quiescent, never allow
  autosuspend, log dmesg before binding. The same `w2_01_camera_usb_guard.sh`
  pattern applies; only the VID:PID changes (`1a6e:089a` for the unbound
  Global Unichip dev board, `18d1:9302` after the Coral firmware loads).
- **MIPI CSI cameras** (Phase 2, optional) — not USB; out of scope for
  this document.
- **PoE IP cameras** (Phase 2, optional) — Ethernet, not USB; out of scope.
- **USB debug bridges** (FT2232, J-Link, ST-Link) — bench-only, used during
  bring-up. They are HID/CDC class devices with negligible current and do
  not need the guard. The global autosuspend disable still applies to them
  for free and removes one more wedge surface.

So the short answer: the **camera and the Coral** are the only two USB
devices in the LifeTrac BOM that justify the full W2-01 guard treatment.
Everything else is either I²C-behind-HID (GPS, IMU), low-current HID/CDC
(MCP2221A, debug bridges), or non-USB (CSI, PoE).

## 8. Pros / Cons of the Deferred Items, and What Each Buys Us

The §3 implementation is what we are about to bench. The following three
items were deliberately not done in this round; this section is the case
for/against doing each one next, and what voltage-drop / wedge headroom
each one actually buys.

A useful framing first: **software cannot raise VBUS.** The C2's first-
attach inrush, the sensor self-cal current, and the isochronous-streaming
draw are all electrical events. Software's job is purely to (a) keep those
events from stacking on top of each other and (b) keep Linux from
mishandling the resulting transients. The numbers below are best-effort
estimates of the *avoided* concurrent load, not a reduction in any single
device's peak.

### 8.1. Boot-time `OSTREE_KERNEL_ARGS=usbcore.autosuspend=-1`

**What it does.** Bakes the autosuspend-disable into the production LmP
factory image so it is in effect before any USB device enumerates,
including the C2 if it is plugged at boot.

**Pros.**
- Closes the only remaining hole in the runtime-write strategy: if the
  C2 is plugged in at power-on, the runtime write happens too late to
  affect the C2 itself. The factory arg fixes this for free.
- Once shipped, the bench operator never needs to think about the cold-
  plug dance again. The guard becomes idempotent rather than load-bearing.
- Covers the MCP2221A, the Coral, and any future USB peripheral with no
  per-device work.
- Survives reboots, power cycles, and field operators who do not know
  about the guard script.

**Cons.**
- Touches the Foundries factory pipeline and requires a new LmP build +
  OTA. That is a real production change with rollback implications, not a
  bench tweak.
- Suppresses runtime PM globally for every USB device on the host. On a
  fielded tractor that is fine because we are not battery-constrained, but
  it is worth documenting.
- Cannot be tested in isolation from the rest of the LmP image cycle.

**Voltage-drop headroom bought.** Indirectly large but not measurable in
millivolts. The avoided wedge is not a sag event; it is the autoresume
pulse on a previously-suspended device, which our bench traces show as a
~50–100 mA step on top of whatever else is happening at that instant.
Worth maybe ~20–50 mV of avoided concurrent IR drop on a typical Max
Carrier USB-A trace + cable. The real value is reliability, not voltage.

**Verdict.** Do this when the next LmP image build happens anyway. Not
worth a one-off image cycle just for this; the runtime guard plus the
udev rule cover ~95 % of the cases.

### 8.2. Systemd oneshot for runtime `usbcore.autosuspend=-1`

**What it does.** A small unit that writes `-1` to the autosuspend sysfs
file early in boot, before any USB hot-plug. Pure stopgap until §8.1
ships.

**Pros.**
- Local to the X8; no Foundries pipeline change.
- Survives reboots without operator action.
- Covers the case where the C2 is plugged at boot.
- Trivially reversible (`systemctl disable`).

**Cons.**
- Race risk: if the unit runs after the kernel has already enumerated the
  C2 (because the camera came up faster than the unit), it does nothing
  for the C2. The avoidance research note already documents that the LmP
  systemd boot order is fast enough for this to be a real concern.
- Two sources of truth for the same setting (unit + guard) is mildly
  fragile; future maintainers can disagree.
- Does not survive a re-flash; needs to be in the recipe to persist.

**Voltage-drop headroom bought.** Same as §8.1 in practice (~20–50 mV of
avoided concurrent IR drop at the autoresume pulse), but only on reboots
where the unit wins the race.

**Verdict.** Worth shipping as a stopgap if §8.1 is more than a sprint
away, or if we want a defense for engineers re-flashing without the
factory arg. Skip otherwise — the guard already covers the bench flow.

### 8.3. Auto-install of `99-w2-01-c2.rules` and `lifetrac-no-usb-audio.conf`

**What it does.** Have the orchestrator (or a small bench helper)
push the udev rule and the modprobe blacklist to the X8 once, then
`udevadm reload` and `rmmod snd_usb_audio`, so future C2 plug events are
mitigated without running the guard script.

**Pros.**
- Removes the per-bench-run guard step. After install, plugging the C2
  is enough to get `power/control=on` + no `snd-usb-audio` on its own.
- Both files are tiny, vendor-specific, and easy to audit.
- The udev rule is the only mechanism that catches a C2 hot-plug AFTER
  the guard has already exited. Today, if the operator unplugs and
  re-plugs the camera mid-session, the new enumeration uses the kernel
  default until the guard is re-run. With the rule installed, every
  enumeration is mitigated.

**Cons.**
- Adds persistent state to the X8. We lose the "just re-flash and start
  clean" property unless the install step is part of the recipe.
- The blacklist is a sledgehammer: every USB audio device is blocked.
  Today LifeTrac has no other USB audio device, so this is fine. If the
  bench ever uses a USB audio dongle for debugging, the blacklist needs
  a per-VID/PID exception.
- One more thing to remember to install on a new X8.

**Voltage-drop headroom bought.** The udev rule itself does not change
voltage; the avoided concurrent load is the snd-usb-audio binding step,
which is roughly ~10–20 mA of isochronous bandwidth reservation plus an
alt-setting transient. On a typical Max Carrier USB-A trace + cable, that
maps to perhaps ~10–30 mV of avoided VBUS sag during the C2's already-
busy attach window. Small in absolute terms, but the timing is the worst
possible (during enumeration), so it stacks against the inrush.

**Verdict.** Yes — but as a one-time bench install with documented
reversal, not as part of the orchestrator's hot path. Adding it to a
"`provision_x8.sh`" alongside the `usermod -aG video fio` line is the
right home.

### 8.4. Total voltage-drop headroom budget

Putting numbers on the whole stack, with all three deferred items shipped
on top of §3:

| Mitigation                                 | Avoided concurrent VBUS load | Approx. avoided sag* |
| ------------------------------------------ | ---------------------------: | -------------------: |
| `usbcore.autosuspend=-1` (any source)      | ~50–100 mA autoresume pulse  | ~20–50 mV            |
| `snd-usb-audio` blacklist / unbind         | ~10–20 mA + alt-setting txn  | ~10–30 mV            |
| Authorization gate + 500–800 ms settle     | sequencing only              | ~50–150 mV†          |
| Start-small MJPEG (640×480 first)          | ~50–150 mA streaming current | ~30–80 mV            |
| **Total software-only headroom**           |                              | **~110–310 mV**      |
| Powered hub (hardware)                     | removes the entire C2 load   | ~300–800 mV          |

\* Estimates assume ~150–300 mΩ total resistance from the X8 5 V rail
through the Max Carrier traces, the SMSC USB hub, and the locking USB-A
to USB-C cable to the C2's input. Order-of-magnitude only; bench
measurement on the next session would tighten these.

† The authorization gate's "headroom" is not a current reduction; it is a
deferral. By the time we authorize, the X8's bulk caps have had ~500 ms
to recharge from whatever was happening at attach, so the *peak*
concurrent draw is lower even though the average is the same.

**Bottom line on the numbers.** Software can plausibly avoid on the
order of **100–300 mV** of additional VBUS sag during the C2 attach
window, mostly via sequencing rather than current reduction. A powered
hub removes 2–3× more sag than the entire software stack combined and is
therefore still the right structural answer. The software work is what
keeps the X8 Linux side from mishandling whatever transient remains.

## 9. Production Operating Model: C2 Permanently Attached Across Power Cycles

§3 through §8 are framed around the bench scenario where an operator hot-
plugs the C2 into a running X8. In production the camera is permanently
cabled to the tractor X8 and is energised every time the X8 powers on.
That changes which mitigations matter and introduces a different set of
failure modes that are not addressed elsewhere in this document.

### 9.1. What changes vs. the bench scenario

The dominant enumeration path is now **cold boot**, not hot-plug. The
specific race that wedged us at the bench — `usbcore.autosuspend=2`
firing while `snd-usb-audio` was still probing a freshly hot-plugged
composite device — is much less likely during cold boot, but should not
be treated as impossible. USB hub enumeration, driver binding, runtime
PM setup, and udev/module-load can still overlap during early boot. The
production conclusion is the same, but the reason is sharper: **the
runtime sysfs write the W2-01 guard performs is too late**. By the time
any userspace runs, the C2 may already have enumerated under the kernel
default. Every guarantee has to be in place *before* the kernel sees the
C2.

| Concern                          | Bench (today)                          | Production (C2 always attached)                         |
| -------------------------------- | -------------------------------------- | ------------------------------------------------------- |
| Wedge trigger to defend against  | hot-plug into running kernel           | lower probability at boot; still validate power cycles  |
| `usbcore.autosuspend=-1`         | runtime sysfs write (volatile)         | **must be in `OSTREE_KERNEL_ARGS`** (§8.1)              |
| `snd-usb-audio` block            | unloaded once per session              | **must be persistent `install … /bin/false`**           |
| Capture service start            | manual / orchestrator                  | **systemd unit ordered After= the `.device` unit**      |
| Recovery from brownout / unplug  | operator replug                        | **udev-triggered restart + watchdog**                   |
| Mechanical interface             | bench USB-A friction fit               | **strain-relieved or screw-locking USB**                |
| Authorization gate               | guard step                             | **not used** — defeats boot ordering, see §9.4          |

### 9.2. Persistent software state required for production

These are the W2-01.D-series TODO items, promoted from "nice to have"
to **required for production deployment**:

1. **`OSTREE_KERNEL_ARGS += "usbcore.autosuspend=-1"`** in the LmP build
   recipe (§8.1, TODO W2-01.D3). This is the only one of the §3 knobs
   that takes effect *before* the C2 is enumerated at boot, which is
   exactly when production needs it. Reviewer note: Setting this
   globally disables runtime PM for the entire USB host controller and
   all downstream ports. This will impose a permanent ~1-2W power
   penalty; confirm the vehicle's 12V/5V power budget can absorb this
   idle load.
2. **`/etc/modprobe.d/lifetrac-no-usb-audio.conf`** with
   `install snd_usb_audio /bin/false`. This is stronger than `blacklist`
   because it also blocks auto-load triggered by a UAC descriptor
   appearing on the bus. Lives in mutable `/etc`, survives OSTree
   deploys. Reviewer note: the checked-in file currently uses only
   `blacklist snd_usb_audio`; production should upgrade it to the
   stronger `install snd_usb_audio /bin/false` form or explicitly record
   why `blacklist` is enough on the target LmP image.
3. **`/etc/udev/rules.d/99-w2-01-c2.rules`** with three jobs:
   - Stable symlink: `SUBSYSTEM=="video4linux", ATTRS{idVendor}=="16d0",
     ATTRS{idProduct}=="0ed4", SYMLINK+="lifetrac-c2"`. Capture binds
     `/dev/lifetrac-c2`, never `/dev/video0` (which can renumber if any
     other V4L device ever appears, e.g. a debug HDMI capture dongle).
     Reviewer note: UVC devices can expose more than one `/dev/video*`
     node, so the final rule should also match only the real capture
     node. Rather than just relying on `ATTR{index}=="0"` (which can be
     fragile if the driver discovers nodes out of order), using
     `ENV{ID_V4L_CAPABILITIES}==":capture:"` is the safest primary
     symlink match.
   - `TAG+="systemd"` to enable the synthesised `dev-lifetrac\x2dc2.device`
     unit for service ordering (§9.3).
   - Use `ENV{SYSTEMD_WANTS}+="lifetrac-camera.service"` or `.device`
     `BindsTo=` wiring for hotplug recovery. Avoid
     `RUN+="/bin/systemctl restart ..."` in udev: udev `RUN` jobs are
     meant to be short-lived, can be killed by the udev worker timeout,
     and can deadlock or race systemd during boot.
   Reviewer note: the checked-in `99-w2-01-c2.rules` currently only sets
   `power/control` and `power/autosuspend_delay_ms`; it does not yet
   create `/dev/lifetrac-c2`, tag the device for systemd, or request the
   camera service.
4. **`provision_x8.sh`** (TODO W2-01.D1): an idempotent installer that
   lays down items 2 and 3, runs `udevadm control --reload-rules`, and
   adds `fio` to the `video` group. Run once per fresh image; safe to
   re-run as a self-heal step.

### 9.3. systemd service topology

```
[multi-user.target]
        │
        └── lifetrac-camera.service
                 Wants=    dev-lifetrac\x2dc2.device
                 Requires= dev-lifetrac\x2dc2.device   # fail fast if camera missing at boot
                 BindsTo=  dev-lifetrac\x2dc2.device   # stop if the device disappears
                 After=    dev-lifetrac\x2dc2.device
                 Restart=  on-failure
                 RestartSec= 2s
                 StartLimitIntervalSec= 60
                 StartLimitBurst= 5
```

The `Requires=` plus `After=` on the synthesised `.device` unit is what
makes systemd *wait for the kernel to enumerate the C2* before starting
capture, instead of racing it. `BindsTo=` is the missing production
piece: it stops `lifetrac-camera.service` when `/dev/lifetrac-c2`
disappears, instead of assuming the capture process notices the USB
remove event by itself. If the device never appears (cable yanked
pre-boot, brownout during enumeration), the service fails cleanly,
journald records why, and the autonomy stack can read service state to
decide whether to continue without vision or block.

For mid-run disconnect / brownout, the udev `ACTION=="add"` rule from
§9.2.3 plus `BindsTo=` and `Restart=on-failure` give automatic recovery
the next time the kernel sees the device. `StartLimitBurst=5` prevents
thrash if the cable is genuinely flapping; once exhausted, the service
parks and the LoRa health beacon ships `cam=wedged`. Reviewer note: When
`BindsTo=` cleanly stops the service during a disconnect,
`Restart=on-failure` will **not** bring it back. Ensure the udev rule
explicitly includes `ENV{SYSTEMD_WANTS}+="lifetrac-camera.service"` so
the auto-recovery is correctly triggered by udev when the camera
re-enumerates.

### 9.4. Why the §3 authorization gate is *not* used in production

The bench guard holds the C2 unauthorized until the system is quiet, then
authorizes it. In production this is actively counterproductive:

- `usbcore.authorized_default=0` would block *every* USB device at boot,
  including the MCP2221A, which the autonomy stack wants up immediately.
- The "system is quiet" trigger that justifies authorization at the
  bench (operator decides) does not exist on a tractor that boots
  unattended. Any chosen trigger (timer, CPU idle threshold, etc.) is
  worse than just letting the kernel enumerate normally.
- Once `usbcore.autosuspend=-1` is in the cmdline, the C2 enumerates
  with autosuspend already disabled. The §3 authorization gate's only
  job — preventing autosuspend from firing during the attach window —
  is already done.

The systemd `Requires=` ordering replaces the authorization gate as the
"don't touch the camera until it's ready" mechanism, and does it without
needing root or per-device sysfs writes.

### 9.5. Boot-time canary: keep the W2-01 guard as a oneshot

Wire the existing `w2_01_camera_usb_guard.sh` into the production image
as a `Before=lifetrac-camera.service` oneshot. Three reasons:

- **Regression detector.** Records `autosuspend`, `snd-usb-audio` state,
  and C2 descriptors *every boot*. If a future kernel or OSTree update
  silently drops one of the knobs, the guard's `verdict=` field flips
  and the LoRa beacon ships the change next packet.
- **Single-byte telemetry.** The `__W2_01_GUARD_BEGIN__` / `__W2_01_GUARD_END__`
  block is already machine-parseable; the health beacon only needs to
  ship one bit (PASS/FAIL).
- **Policy hook.** If `verdict=FAIL` because the camera is missing at
  boot, the service can decide whether to start the autonomy stack
  without vision or refuse to start. That is a product decision, not a
  firmware one, but the canary is what makes it *expressible*.

The guard's runtime sysfs writes become idempotent in this configuration
(autosuspend is already `-1` from the cmdline, snd-usb-audio is already
blocked by modprobe.d). It still records state, which is the entire
point in production. Reviewer note: the current guard exits `0` by
design and its `PASS` condition is intentionally minimal (C2 present +
autosuspend disabled). A production oneshot that gates startup should
wrap the guard, parse the framed status block, and fail the unit if the
policy-relevant fields are wrong (for example C2 missing,
`autosuspend_final` not `-1`, `snd-usb-audio` bound, or no
`/dev/lifetrac-c2` capture node).

### 9.6. Cold-boot power & enumeration concerns (new in production)

This is the failure surface that does *not* exist at the bench, because
at the bench the X8 boots first and the C2 is plugged afterward.

- The C2 draws ~250 mA steady, with a ~400 mA peak during sensor PLL
  lock (~50 ms window).
- At cold boot the i.MX8 SoM, eMMC init, LoRa radio init, and C2
  enumeration all hit the 5 V rail in the same ~500 ms window.
- The voltage-drop budget in §8.4 pegs avoidable software-side losses at
  ~110–310 mV. Cumulative drop with a long bulkhead cable + carrier
  trace + SMSC hub can put VBUS at the camera below the ~4.75 V the
  OV9281 / IMX-class sensors need during PLL lock. The visible symptom
  is enumeration succeeding but first-frame ISP init flaking
  intermittently, which the bench scenario never sees because the X8 is
  already past its boot peak when the operator plugs the C2.
- **Mitigation hierarchy, cheapest first:**
  1. Short USB cable (<300 mm), 22 AWG VBUS pair.
  2. Powered USB hub between carrier and C2 (§5 Bottom Line + §8.4),
     even if the C2 is the only downstream device. Isolates inrush and
     gives the C2 its own clean 5 V; the hub's upstream cable then
     carries only data + ~100 mA enumeration current.
    3. If camera ground isolation is required, choose a **high-speed USB
      isolator** or isolate the camera power/chassis path separately.
      Do not put the ADuM3160 on the C2 UVC data path: the ADuM3160 is a
      low/full-speed isolator and is appropriate for the MCP2221A HID
      branch, not a high-speed UVC camera.

### 9.7. Mechanical interface (single biggest production risk)

USB-A friction fit on a vibrating tractor will fail. Pick one:

- **Screw-lock USB-A** — Kurokesu sells C2 variants with this; the X8's
  Max Carrier USB-A bulkhead does not, so this still leaves the carrier
  end exposed.
- **Strain-relief boot + cable clamp** within 50 mm of both ends. The
  failure point shifts from the connector to the clamp, which is what we
  want.
- **Potted joint** at the camera enclosure penetration if IP-rated.

The Max Carrier USB-A bulkhead is friction-only. Without strain relief
on the carrier end, the bulkhead solder joints become the failure point,
which is much more expensive to repair than a cable.

### 9.8. Production failure modes and responses

| Failure mode                              | Detection                                       | Auto-action                                              | Operator action          |
| ----------------------------------------- | ----------------------------------------------- | -------------------------------------------------------- | ------------------------ |
| C2 missing at boot                        | guard `verdict=FAIL`, `.device` unit times out  | service stays inactive; beacon flags `cam=down`          | inspect cable            |
| Brownout disconnect mid-run               | udev `remove` event; `.device` becomes inactive | `BindsTo=` stops service; udev/systemd starts on re-add   | none if recovered <2 s   |
| USB host wedge (hub `EILSEQ` storm)       | dmesg burst; `Restart=on-failure` exhausts burst| systemd parks service; beacon flags `cam=wedged`         | reboot X8                |
| OSTree update regresses cmdline           | guard reads `autosuspend_initial≠-1`            | beacon flags `cam=policy_drift`                          | rebuild image            |
| Sensor thermal throttle                   | frame timing variance                           | (out of scope for this layer)                            | enclosure ventilation    |

### 9.9. Suggested deliverable order for production hardening

In priority order — none required for *first* bench light, all required
before field deploy:

1. **`provision_x8.sh`** (TODO W2-01.D1) — installs §9.2 items 2 and 3
   plus `usermod -aG video fio`. Idempotent. Lands cleanly on existing
   X8s without an image rebuild.
2. **`lifetrac-camera.service` skeleton** + `dev-lifetrac\x2dc2.device`
   wiring — even if the actual capture binary is a placeholder
   (`cat /dev/lifetrac-c2 > /dev/null`), prove the unit topology and
   `systemd-analyze verify` it before the real capture exists.
3. **`OSTREE_KERNEL_ARGS` patch** to the LmP layer (§8.1, TODO W2-01.D3) —
   the only item that requires a Foundries image rebuild, so it has the
   longest lead time. Start the recipe change in parallel with item 1.
4. **Boot-time guard oneshot** — repackage the existing
   `w2_01_camera_usb_guard.sh` as a systemd unit per §9.5.
5. **Powered-hub spec & cable BOM update** (TODO W2-01.D5) — closes the
   §9.6 hardware gap.
6. **Mechanical strain relief / locking-USB BOM update** — addresses
   §9.7.

Items 1 and 2 are doable on the X8 we already have on the bench
(`2E2C1209DABC240B`) without touching the LmP image, and `systemd-analyze
verify` exercises the unit ordering offline. Item 3 is the only
production-hardening step that blocks on a Foundries build cycle.

### 9.10. Review comments before implementation

These are the review findings to carry into the first production draft:

| Finding | Why it matters | Action |
| ------- | -------------- | ------ |
| Cold boot is lower risk, not zero risk | USB enumeration and driver binding can still overlap early boot; the original text overstated this as impossible | Validate with repeated power cycles with the C2 permanently attached, capturing `dmesg`, `/proc/cmdline`, `lsusb -t`, and service state each time |
| Checked-in audio policy is weaker than §9.2 | `blacklist snd_usb_audio` may not block every module auto-load path; §9.2 calls for `install snd_usb_audio /bin/false` | Update `lifetrac-no-usb-audio.conf` or explicitly document why `blacklist` is sufficient on this LmP image |
| Checked-in udev rule is incomplete for production | It sets USB PM only; it does not create `/dev/lifetrac-c2`, tag the device for systemd, or request the service | Extend or split the rule into a USB-device PM rule and a video4linux symlink/systemd rule |
| UVC may expose multiple video nodes | A plain VID/PID symlink can bind the metadata node instead of the capture node | Match the final rule against `ATTR{index}=="0"`, `ENV{ID_V4L_CAPABILITIES}`, or the observed C2 udev properties |
| Avoid `systemctl restart` directly from udev `RUN` | udev workers are not a reliable place to run long-lived service control | Use `TAG+="systemd"`, `ENV{SYSTEMD_WANTS}`, and service `BindsTo=`/`After=` relationships |
| Unit sketch omits section placement | `Restart=` belongs in `[Service]`, while `StartLimitIntervalSec=`/`StartLimitBurst=` belong in `[Unit]` on systemd versions that support the `Sec` form | Draft the real `.service` file explicitly and run `systemd-analyze verify` on the target image |
| Guard oneshot is a canary, not a hard gate yet | The current script always exits 0 and does not validate every production invariant | Add a small wrapper or guard mode that exits nonzero on policy drift before wiring it as a blocking `Before=` unit |
| ADuM3160 is not suitable for the C2 data path | It is low/full-speed, while the UVC camera needs high-speed USB | Keep ADuM3160 on the MCP2221A branch only; spec a high-speed isolator if the camera itself needs isolation |
| Power numbers are estimates | The 250 mA / 400 mA and 100–300 mV figures are planning values, not measured on the tractor harness | Add a bench task to measure VBUS at the C2 during cold boot and first `STREAMON` with the final cable/hub assembly |
| `BindsTo=` requires `SYSTEMD_WANTS` to recover | When `BindsTo=` cleanly stops the service during a disconnect, `Restart=on-failure` will **not** bring it back | Ensure the udev rule includes `ENV{SYSTEMD_WANTS}+="lifetrac-camera.service"` so udev starts it on re-attach |
| Global autosuspend has a system power penalty | Setting `usbcore.autosuspend=-1` keeps the USB host controller and **all** hub ports active permanently | Verify the tractor 12V/5V power budget can absorb an extra ~1-2W of idle draw from the un-suspended host |
| V4L2 capability matching is safer than index | `ATTR{index}=="0"` can be fragile if the driver discovers nodes out of order | Use `ENV{ID_V4L_CAPABILITIES}==":capture:"` (provided by `60-persistent-v4l.rules`) as the primary symlink match |

The first implementation pass should therefore be: update the two policy
files to match the stronger production notes, draft the `.service` unit
with `BindsTo=`, then run a cold-boot validation loop before promoting
the `OSTREE_KERNEL_ARGS` change into the image recipe.

## 10. Consolidated Implementation Plan

This section folds every reviewer note from §9.2, §9.3, §9.5, and §9.10
together with the self-review of those notes into a single, ordered,
actionable plan. Where §9.10 and §9.2/§9.3 overlap, **this section is
authoritative**; §9.10 is retained as the audit trail of how we got
here.

### 10.0. Corrections that supersede earlier sections

These fix mistakes introduced into the earlier reviewer notes
themselves and should be considered the final form:

1. **`ID_V4L_CAPABILITIES` match must be a glob and not the only
  discriminator.**
   `v4l_id` emits a colon-delimited list (`:capture:`,
   `:capture:video_output:`, etc.). Use
   `ENV{ID_V4L_CAPABILITIES}=="*:capture:*"`, **not** `==":capture:"`.
    Bench evidence on 2026-05-15 also showed that all four C2 video nodes
    can satisfy the broad capability match, so combine the glob with
    `ATTR{index}=="0"` for `/dev/lifetrac-c2` and the systemd trigger.
2. **Drop the "~1-2 W" idle-power figure.** That number is itself an
   estimate. Replace with: "non-zero idle power penalty on the USB
   host; measure on the bench before committing to the global cmdline
   form." The actual delta on this i.MX8 carrier with one C2 +
   MCP2221A is expected to be a few hundred mW, but is unverified.
3. **Treat `usbcore.autosuspend=-1` as conditional, not mandatory.**
   The original wedge required `snd_usb_audio` probing *and*
   autosuspend firing in the same window. If §10.1.B (`install
   snd_usb_audio /bin/false`) holds across every code path, the
   cmdline arg becomes belt-and-suspenders. Decision gate is
   §10.3.V3.
4. **`StartLimitIntervalSec=` / `StartLimitBurst=` placement.** On
   the systemd version shipped with LmP/Foundries (≥230), both
   directives belong in `[Unit]`. `Restart=` and `RestartSec=` belong
   in `[Service]`. There is no version-dependent ambiguity on this
   target.
5. **§9.8 "Brownout disconnect mid-run" row is conditional on
   §10.1.C.3.** Auto-recovery only happens if the udev rule includes
   `ENV{SYSTEMD_WANTS}+="lifetrac-camera.service"`; without it,
   `BindsTo=` cleanly stops the unit and nothing brings it back.
6. **Bare-host Python is not a production capture dependency.** The
  bench X8 image has a deliberately small Python runtime; missing
  modules such as `_ctypes`/`ctypes`, `fcntl`, `mmap`, `pip`, and common
  media packages are normal on Foundries LmP host images. The Python
  first-light helper remains bench scaffolding only. Production camera
  capture must run in the application container stack, and low-level
  bench frame capture should use a static aarch64 helper rather than
  host Python.
7. **`adb shell` does not honour `/etc/group` for the `fio` user.**
   adbd on the LmP image hands the `fio` shell an Android-style
   supplementary group set (`1003,1004,3001…`) that excludes
   `video`/`plugdev`/`docker` even after `usermod -aG video fio`
   succeeds. Bench probes that need to open `/dev/lifetrac-c2` from
   adb must elevate via `sudo -S` (the V3 stress harness does this);
   the production capture path runs via systemd + docker compose and
   does see `/etc/group`, so this quirk does not affect deployment.

### 10.1. Deliverables

Each deliverable is a single artifact with an owner-visible name, the
file or location it lives in, and the acceptance test that proves it
works.

**A. `provision_x8.sh` (TODO W2-01.D1)**

- *Location:* `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/provision_x8.sh`
- *Does:* lays down B, C, D; runs `udevadm control --reload-rules &&
  udevadm trigger`; runs `usermod -aG video fio`; idempotent.
- *Acceptance:* second invocation is a no-op (`diff` of `/etc` before
  and after returns empty); `getent group video` shows `fio`.

**B. `/etc/modprobe.d/lifetrac-no-usb-audio.conf`**

- *Content:* `install snd_usb_audio /bin/false`
  (plus a one-line comment pointing back to this section). The checked-in
  helper now uses this stronger `install` form; the older `blacklist
  snd_usb_audio` form is retained only as historical context in earlier
  sections of this note.
- *Acceptance:* after a cold boot with the C2 attached,
  `lsmod | grep snd_usb_audio` is empty and
  `find /sys/bus/usb/drivers/snd-usb-audio -maxdepth 1 -type l` shows
  no bound interface.

**C. `/etc/udev/rules.d/99-w2-01-c2.rules`**

Split into three rule lines so each concern is independently auditable:

1. **USB-device PM (existing concern):**
   `SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="16d0", ATTR{idProduct}=="0ed4",
    TEST=="power/control", ATTR{power/control}="on",
    TEST=="power/autosuspend_delay_ms", ATTR{power/autosuspend_delay_ms}="-1"`
2. **V4L capture symlink + systemd tag:**
   `SUBSYSTEM=="video4linux", ATTRS{idVendor}=="16d0",
  ATTRS{idProduct}=="0ed4", ATTR{index}=="0",
  ENV{ID_V4L_CAPABILITIES}=="*:capture:*", SYMLINK+="lifetrac-c2",
  TAG+="systemd"`
3. **Hotplug recovery:**
   `SUBSYSTEM=="video4linux", ACTION=="add", ATTRS{idVendor}=="16d0",
  ATTRS{idProduct}=="0ed4", ATTR{index}=="0",
  ENV{ID_V4L_CAPABILITIES}=="*:capture:*", ENV{SYSTEMD_WANTS}+="lifetrac-camera.service"`
- *Acceptance:* after cold boot, `readlink /dev/lifetrac-c2` resolves;
  `systemctl status dev-lifetrac\\x2dc2.device` is `active (plugged)`;
  unplug + replug restarts `lifetrac-camera.service` within 2 s.

**D. `lifetrac-camera.service`**

```
[Unit]
Description=LifeTrac C2 camera service
Wants=dev-lifetrac\x2dc2.device
Requires=dev-lifetrac\x2dc2.device
BindsTo=dev-lifetrac\x2dc2.device
After=dev-lifetrac\x2dc2.device
StartLimitIntervalSec=60
StartLimitBurst=5

[Service]
Type=simple
Environment=LIFETRAC_CAMERA_COMPOSE=/opt/lifetrac/compose-apps/lifetrac-camera/docker-compose.yml
ExecStart=/bin/sh -lc 'if [ -f "$LIFETRAC_CAMERA_COMPOSE" ] && command -v docker >/dev/null 2>&1; then exec docker compose -f "$LIFETRAC_CAMERA_COMPOSE" up --remove-orphans; fi; echo "lifetrac-camera: compose app not installed; holding C2 device sentinel"; while [ -e /dev/lifetrac-c2 ]; do sleep 5; done; exit 1'
Restart=on-failure
RestartSec=2s

[Install]
WantedBy=multi-user.target
```

- *Acceptance:* before the compose app exists, the unit stays active as
  a device-presence sentinel and does not open `/dev/lifetrac-c2`. After
  TODO W2-01.D5 lands, `systemd-analyze verify lifetrac-camera.service`
  is silent on the target image and `journalctl -u lifetrac-camera`
  shows the foreground compose process running with `/dev/lifetrac-c2`
  mapped into the camera container.

**E. `w2_01_camera_usb_guard_gate.sh` (hard-gate wrapper)** — *shipped 2026-05-15*

- *Wraps* the existing `w2_01_camera_usb_guard.sh`, parses its
  `__W2_01_GUARD_BEGIN__`/`__W2_01_GUARD_END__` block, and exits
  non-zero if any of: C2 missing, `autosuspend_final` not effectively
  disabled (per §10.3.V3 outcome), `snd_usb_audio` bound,
  `/dev/lifetrac-c2` symlink missing or not pointing at
  video-index0. The gate also accepts per-device PM equivalence
  (`power/control=on` + `power/autosuspend_delay_ms=-1`) so it does not
  hard-require deliverable F.
- *Wired in as* a `Before=lifetrac-camera.service` oneshot.
- *Acceptance:* deliberately removing the modprobe.d file and
  rebooting causes the gate unit to fail and `lifetrac-camera.service`
  to refuse to start.
- *Bench result 2026-05-15:* `gate=PASS` on the bench X8 across two
  consecutive V3 stress runs (50 + 200 iterations); evidence under
  `DESIGN-CONTROLLER/bench-evidence/w2_01_v3_stress_2026-05-15_114302/`
  and `…_114350/`.
- *Side-finding:* the original `w2_01_camera_usb_guard.sh` had a
  cosmetic glob bug (`${ifpath}driver` missing a slash) that mis-
  reported all C2 interfaces as `<unbound>`. Fixed in the same change
  that shipped E.

**F. `OSTREE_KERNEL_ARGS += "usbcore.autosuspend=-1"` (TODO W2-01.D3)** — *shelved 2026-05-15 pending colder reproduction attempt*

- *Conditional on §10.3.V3.* Only promote into the LmP recipe if the
  bench validation in V3 shows the wedge can still be triggered with
  B+C+D+E in place. If V3 cannot reproduce the wedge, this deliverable
  is downgraded to "documented but not shipped".
- *Bench result 2026-05-15:* 0/250 wedge reproductions across V3
  stress (50 + 200 iters of sysfs descriptor reads + V4L2 device
  open). With B+C+D+E in place the trigger pattern no longer wedges
  the host, so F stays shelved. Re-evaluate after the next cold-boot
  campaign or if V5 surfaces a VBUS sag.
- *Acceptance (if revived):* `cat /proc/cmdline` on a freshly imaged
  X8 contains `usbcore.autosuspend=-1`.

**G. Hardware BOM updates**

- Powered USB hub between Max Carrier and C2 (closes §9.6).
- Strain-relief boot + cable clamp within 50 mm of both connector ends
  (closes §9.7). Screw-lock USB-A on the C2 end if available.
- Decision still open: high-speed USB isolator on the camera path
  *only if* a galvanic-isolation requirement materialises. The
  ADuM3160 is **not** a candidate here; it stays on the MCP2221A
  branch.
- *Acceptance:* BOM PR merged; mechanical drawing references the
  clamp locations.

**H. Static aarch64 bench capture helper (TODO W2-01.D4a)**

- *Why:* the host LmP Python runtime is intentionally stripped and is
  not a reliable substrate for V4L2 ioctl capture tests.
- *Content:* either a static `v4l2-ctl` build or a tiny dedicated C
  helper that enumerates C2 formats and grabs one MJPEG/YUYV frame from
  `/dev/lifetrac-c2` without Python, OpenCV, GStreamer, or FFmpeg.
- *Acceptance:* `adb push` to `/tmp`, run as `fio` after `usermod -aG
  video fio`, produces a non-empty frame file and exits non-zero on
  open/ioctl/STREAMON errors without wedging the X8.

**I. Containerized camera runtime (TODO W2-01.D5)**

- *Location:* `/opt/lifetrac/compose-apps/lifetrac-camera/` on the X8
  image, with source in the tractor controller tree.
- *Does:* owns the production capture/encoding process inside a
  container, with `/dev/lifetrac-c2` passed through as a device and the
  host service in D supervising the foreground `docker compose up`.
- *Acceptance:* service startup launches the compose app, the container
  can open `/dev/lifetrac-c2`, and the health path reports frame timing
  without depending on any host Python module.

### 10.2. Dependencies and ordering

```
A (provision_x8.sh) ──┬── B (modprobe.d)
                      ├── C (udev rules)
                      └── usermod -aG video fio
                                │
                                ▼
                          D (.service unit) ── systemd-analyze verify
                                │
                                ▼
                            V1..V4 USB-policy validation (§10.3)
                                │
                                ▼
                          E (guard gate)  ── ties D start to canary PASS
                                │
                                ▼ (conditional, §10.3.V3 outcome)
                          F (OSTREE_KERNEL_ARGS) ── needs LmP rebuild

                  H (static capture helper) and I (containerized runtime) run in parallel
                  after V1/V2 prove the USB policy. H is the bench frame-grab unblocker;
                  I is the field deployment path.

G (hardware BOM) runs in parallel with A..F; gates field deploy, not
bench validation.
```

A through E land on the existing X8 (`2E2C1209DABC240B`) without an
image rebuild. F is the only item that costs a Foundries build cycle,
and §10.0.3 / §10.3.V3 are explicitly designed to decide whether F is
worth that cost.

### 10.3. Bench validation matrix

Every validation runs on the existing X8 with the C2 permanently
attached unless noted. Each row produces a recorded artifact under
`DESIGN-CONTROLLER/bench-evidence/w2_01_production_<YYYY-MM-DD>/`.

| ID | Validation                              | Apparatus                                                    | Pass criterion                                                       |
| -- | --------------------------------------- | ------------------------------------------------------------ | -------------------------------------------------------------------- |
| V1 | Cold-boot enumeration, 20 cycles        | Reboot/power cycle loop + `run_w2_01_bench_validation.ps1 -SysfsOnly` | C2 enumerates, `/dev/lifetrac-c2` resolves, and no USB wedge every cycle |
| V2 | `snd_usb_audio` blocked across boots    | `lsmod`, `dmesg \| grep snd_usb_audio` after each V1 cycle    | zero matches                                                         |
| V3 | Wedge reproducibility *with B+C+D+E*    | `run_w2_01_v3_stress.ps1` — sysfs descriptor reads + V4L2 open loop, no kernel cmdline change | if reproduces → ship F; if 0/50 reproductions → keep F shelved. **2026-05-15: 0/250 reproductions across two runs (50 + 200 iters), F shelved.** |
| V4 | Hotplug recovery                        | unplug C2 mid-run, wait 5 s, replug                          | `systemctl status` returns to `active (running)` within 2 s of replug |
| V5 | VBUS at the camera under cold boot      | USB power meter inline, scope on 5V                          | VBUS ≥ 4.75 V through PLL-lock window                                |
| V6 | Idle-power delta from `autosuspend=-1`  | DC current clamp on 5 V rail; measure with and without F     | recorded delta in mW, replaces the placeholder figure in §10.0.2     |

V1, V2, V4 must pass before E is merged. V3 is the gate for F. V5 and
V6 inform G and §10.0.2 respectively but do not block A..E.

### 10.4. Decisions that need a human before merging

These cannot be auto-resolved from the data already in this document:

1. **Should the autonomy stack start without vision** if E reports
   `verdict=FAIL` at boot? Product decision; informs whether D uses
   `Requires=` (current draft, hard-fails) or `Wants=` (degrades).
2. **Powered-hub vendor and cable spec** — needs a part number, not
   just "powered USB hub". Drives G.
3. **Whether to ship F in the next image** — answered by V3 outcome.
4. **Whether `blacklist` is acceptable instead of `install`** for
   §10.1.B on the target LmP image — answered by inspecting which
   subsystems on this image can trigger module auto-load (e.g. via
   `kmod` aliases vs. udev `MODALIAS=`).

### 10.5. What §9.10 is now for

§9.10 stays as the unedited list of review findings that produced this
plan. It is no longer an action list. New findings discovered during
V1–V6 should be appended to §9.10 with a date, then either folded into
§10.1/§10.3 or explicitly deferred.
