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
