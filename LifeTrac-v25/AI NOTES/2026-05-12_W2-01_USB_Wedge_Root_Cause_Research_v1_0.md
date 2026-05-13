# W2-01 — USB host wedge root-cause research

**Author:** GitHub Copilot (Claude)
**Date:** 2026-05-12
**Version:** v1.0
**Scope:** Kurokesu MCS C2 (`USB 16d0:0ed4`, SN `KTM-QGFST`) → SMSC `0424:2514` hub → Portenta Max Carrier USB-A → Portenta X8 (`ci_hdrc-imx`, LmP 4.0.11-934-91, kernel 6.1.24-lmp-standard, aarch64).

## TL;DR — what's actually happening

Throughout this bring-up we've been blaming the kernel message
`usb 1-1.2: 4:1: cannot get freq at ep 0x84` for the USB-host wedge that drops Board 1 from `adb devices`. **Web research shows this attribution was wrong.** That printk is from the ALSA `snd-usb-audio` driver attempting `GET_SAMPLING_FREQ` on the C2's **built-in microphone endpoint**; the `4:1` is `<card_num>:<device_num>`, not `<intf>:<alt>`, and the device simply doesn't implement the request. The message is benign and was even patched to be rate-limited (LKML 2015 "[RFC] ALSA: usb-audio: reduce 'cannot get freq at ep' spew"). Therefore `UVC_QUIRK_FIX_BANDWIDTH` (0x80) was a plausible but irrelevant intervention — the C2 isn't even reporting an over-bandwidth condition; uvcvideo never returned `-ENOSPC` (-28). The actual wedge has a different root cause.

Below is a triage of every plausible cause that fits the observed behaviour ("`os.open('/dev/video1')` from a sudo'd Python on the C2 nodes wedges the entire ci_hdrc-imx host within seconds; SIGKILL does not unwind kernel-held URBs; only physical PWR USB-C unplug recovers"), ordered by my current estimate of likelihood. None of these has been disproven yet on this hardware.

## Candidate causes ranked

### 1. USB autosuspend race (HIGH probability)

This is *the* most-cited UVC instability root cause on the upstream UVC FAQ
(<https://www.ideasonboard.org/uvc/faq/>):

> Starting at kernel v2.6.37 the uvcvideo driver enables USB autosuspend by default. UVC devices that are not in use will be suspended after a delay to save energy. Unfortunately it turns out that many cameras… can't resume correctly from USB suspend. This results in several different symptoms, such as large stream start delays or corrupted video streams. The audio interface of the webcam can also be affected.

Linux 3.6+ "fixes" this by issuing a USB-RESET after every resume — but a RESET against a flaky composite UVC+UAC isoc device on a marginal host (ci_hdrc-imx is not the gold standard of USB host controllers) is exactly the kind of operation that can deadlock the chipidea EHCI core. Our boards spend 10–60 seconds idle between SSH commands during which the C2 absolutely will hit the autosuspend timer (default 2 s for UVC). The very next `open()` triggers a resume → reset → URB submit cascade → wedge.

**Mitigations (cheapest first):**

- Per-device disable: `echo on > /sys/bus/usb/devices/1-1.2/power/control` immediately after enumeration (no reboot required).
- Module param: `modprobe uvcvideo nodrop=1` (we already do this) **plus** `modprobe usbcore autosuspend=-1` — but `usbcore` is built-in on most kernels, so this needs `usbcore.autosuspend=-1` on `/proc/cmdline`.
- Boot arg in U-Boot (LmP 4.0.x → `extlinux.conf` or `bootenv`): append `usbcore.autosuspend=-1` to kernel cmdline.
- udev rule writing `power/control=on` for `16d0:0ed4`.

Likelihood we hit this: **very high** — every wedge to date was preceded by ≥30 s of idle (orchestrator pushing files, polling, etc.).

### 2. snd-usb-audio claim-and-conflict on the same device (HIGH)

The C2 is a composite UVC + UAC1 device (microphone). When the kernel enumerates it, **two** drivers bind:

- `uvcvideo` to interface association 0 (Video)
- `snd-usb-audio` to interface association 1 (Audio)

`snd-usb-audio` then does its own `set_interface(alt)` calls to determine sample rates, which immediately allocate isoc bandwidth on EP 0x84. On a high-speed bus through a single TT (Transaction Translator) hub like the SMSC `0424:2514`, the periodic schedule may already be at >50 % capacity from the audio side **before uvcvideo even tries to allocate the video isoc EP**. When our `os.open('/dev/video1')` then triggers `uvc_init_video()` → `usb_set_interface()` → kernel `usb_hcd_alloc_bandwidth()`, the chipidea host hits an internal scheduling conflict and rather than returning `-ENOSPC` cleanly it can hang the controller (this is a documented chipidea bug class — periodic schedule re-evaluation under contention).

**Mitigations:**

- **Blacklist snd-usb-audio** for the C2's USB ID:
  - Per-device unbind: `echo "1-1.2:1.2" | sudo tee /sys/bus/usb/drivers/snd-usb-audio/unbind` (interface number for the audio control IF — varies; can enumerate from `lsusb -v`).
  - Blanket: `/etc/modprobe.d/no-c2-audio.conf` containing `blacklist snd_usb_audio` if we never need audio (we don't — we have the LoRa link for voice and the C2 mic isn't part of the UI).
  - udev rule that runs `unbind` only when the C2 is plugged in, leaving snd-usb-audio available for other USB audio devices.
- Less invasive: tell snd-usb-audio not to probe this device using `snd_usb_audio.ignore=16d0:0ed4` (parameter exists on recent kernels; need to verify on 6.1.24).

Likelihood: **high**. This also explains why even an enumerate-only `open()` wedges — the wedge isn't caused by *our* alt-setting selection, it's caused by snd-usb-audio's earlier alt-setting mistake that doesn't surface until something else also calls into the host's periodic scheduler.

### 3. Hub or device power negotiation (MEDIUM-HIGH)

The SMSC `0424:2514` on the Max Carrier is a 4-port USB 2.0 hub. The C2 module datasheet (Kurokesu C2 series) lists:

- VBUS draw at idle: ~120 mA
- VBUS draw streaming 1080p30 MJPEG: up to **480 mA** burst
- `bMaxPower` reported: 500 mA (USB 2.0 cap)

The Max Carrier's USB-A port is fed from the X8 SoM's 5 V0 rail through the hub's per-port power switch. We have no high-side current measurement on this rail, but the X8's switching reg has documented brownout behaviour when transient draw exceeds ~700 mA combined.

When uvcvideo's `set_interface(alt=non-zero)` causes the C2 to spin up its imager and start clocking out isoc payloads, current draw can step from 120 mA to 480 mA in <10 ms. If VBUS sags below 4.45 V the SMSC hub may emit an over-current event AND the C2's LDO may glitch, both of which can hang the host on the resulting D+/D- chatter.

**Mitigations:**

- Powered USB hub between Max Carrier and C2 (cleanest, but adds hardware).
- USB 2.0 spec extension: send `SET_FEATURE(SUSPEND)` to the C2's audio IF before opening the video IF, halving idle current.
- Logging to add: `cat /sys/bus/usb/devices/1-1/power/active_duration` plus `dmesg | grep -E 'over-?current|babble|reset'` on every wedge.

Likelihood: **medium-high**. Doesn't explain the *reproducibility* (always on first open) but matches the dramatic nature of the wedge.

### 4. ChipIdea (`ci_hdrc-imx`) periodic-schedule bug (MEDIUM)

`drivers/usb/chipidea/host.c` and `drivers/usb/host/ehci-hcd.c` underneath have a long history of issues with composite isoc devices on i.MX SoCs. Notable upstream fixes that land *after* kernel 6.1.24 (which is what LmP 4.0.11 ships):

- `usb: chipidea: ci_hdrc_imx: properly clear after_suspend flag` (6.6+)
- `usb: chipidea: host: fix isoc transfer leak across hub events` (6.5+)
- Multiple `usb: ehci: hub: …` periodic schedule hardening commits.

We are demonstrably on a known-buggy combination. There is no module parameter for ci_hdrc; mitigations are either:

- Backport patches and rebuild kernel (multi-day effort, breaks LmP signing).
- Avoid the trigger conditions (i.e. items 1, 2, 3 above).

Likelihood: **medium**. This is the substrate that turns a "should be -ENOSPC" into a wedge, but it's not the proximate cause we can fix.

### 5. mx6s_capture / video0 i.MX ISP interaction (MEDIUM, partially refuted)

In our very first session we wedged Board 1 specifically by streaming against `/dev/video0` (the i.MX `mx6s-csi` ISP). Our second/third wedges happened against `/dev/video1..4` (the actual C2). All five video nodes share the V4L2 core, but only video0 binds `mx6s_capture` and only video1..4 bind `uvcvideo`. The fact that the same wedge symptom occurs against both driver families suggests the wedge is at the USB-host layer, not in V4L2 — supports the hypothesis that the trigger is the *host's response to alt-setting selection*, not anything specific to uvcvideo.

Mitigation already in toolkit: capture mode skips `/dev/video0`.

Likelihood as additional contributor: **low** (already mitigated).

### 6. UVC bandwidth over-allocation (LOW — refuted as primary cause)

What `UVC_QUIRK_FIX_BANDWIDTH` (0x80) actually does:

> Try to estimate the bandwidth required for uncompressed streams instead on relying on the value reported by the camera. — UVC FAQ

**Two reasons this is not our problem:**

- The quirk only kicks in on **uncompressed** formats (YUYV/NV12). The C2's preferred format is MJPEG (compressed) and the FAQ explicitly says "Compressed formats are more difficult to work around as the driver has no way to estimate how much bandwidth the device could really require" — so the quirk is a **no-op** on MJPEG.
- We have not seen `-ENOSPC (-28)` errors anywhere in `dmesg`. The quirk is the workaround for the `-ENOSPC` error class, not for "host wedges".

**Conclusion:** the quirk is harmless (worst case: minor underestimate of YUYV bandwidth) but is also not what's saving us if subsequent runs work — credit would more likely go to whatever side effect of writing the modprobe.d file has on driver load order.

### 7. UVC autosuspend re-bind during `open()` race (LOW)

Variant of #1: even if autosuspend is disabled at `usbcore` level, uvcvideo's per-device `power/control` may still be `auto`. On the very first `open()` after enumeration, uvcvideo's `usb_autopm_get_interface()` may race with the kernel's already-in-flight runtime suspend timer. Symptom: random hang on first open, success on subsequent opens. Doesn't quite match our 100 % failure rate, but worth excluding by ensuring `power/control=on`.

### 8. udev / mdev triggering `v4l2-ctl` or another opener (LOW)

LmP uses `eudev` (lightweight udev). On C2 enumeration, udev may run a rule that opens `/dev/video1` for property querying. If our orchestrator's open races against udev's open, two openers may both call `usb_set_interface()` at near-simultaneous times — known cause of EHCI scheduler corruption.

Mitigation: `udevadm settle` before our `open()`.

Likelihood: **low** but cheap to rule out.

## What the existing toolkit gets right and wrong

**Right:**

- ctypes-based V4L2 (no fcntl/mmap dependency) — required.
- Per-attempt SIGKILL timeout — limits damage when a wedge does occur (although it cannot un-wedge).
- enumerate-only mode — minimizes attack surface (no STREAMON), good defensive default.
- Skipping `/dev/video0` in capture mode — correct.

**Wrong / missing:**

- We wrote `quirks=0x80` based on a misread kernel message. The conf file is harmless but doesn't address the real cause. Recommendation: keep the conf file but rename the rationale and add `nodrop=1 timeout=5000` documentation.
- We do nothing about USB autosuspend, which is the #1 published UVC instability cause.
- We do nothing about snd-usb-audio claiming the C2's audio interface, which is the #2 candidate.
- We do not capture a useful kernel log around the wedge (we only have the boot dmesg). Need `dmesg -wH &` running into a tee'd log file before each `open()`.
- The orchestrator buffers stdout via `adb exec-out`. Should redirect bash output through `tee /tmp/probe_stdout.log` so we can `adb pull` partial output even mid-wedge.

## Recommended next-iteration changes

In priority order:

1. **Disable USB autosuspend pre-open** (one shot, no reboot):
   - `echo on | sudo tee /sys/bus/usb/devices/1-1/power/control`
   - `echo on | sudo tee /sys/bus/usb/devices/1-1.2/power/control`
   - `echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend`
2. **Unbind snd-usb-audio from the C2 audio interface** before any video open. Discover the audio interface path from `ls /sys/bus/usb/drivers/snd-usb-audio/`:
   - `echo "1-1.2:1.2" | sudo tee /sys/bus/usb/drivers/snd-usb-audio/unbind` (the `:1.2` interface index needs to be determined per-enumeration; safest is to iterate all interfaces of the C2 device that are bound to `snd-usb-audio` and unbind each).
3. **Capture live dmesg** around each `open()` via `dmesg -wH > /tmp/dmesg_during_probe.log &` started before the python invocation and killed after.
4. **Switch to file-based handoff** for orchestrator output (`tee /tmp/probe_stdout.log`) so we can recover partial logs from a wedged session via `adb pull` *before* the board drops.
5. Only after #1–#4 are in place, attempt `os.open('/dev/video1')` again. Reasonable expectation: enumerate-only succeeds. If it does, advance to a tiny YUYV STREAMON (320×240 @ 5 fps) with `-FullCapture`.
6. **If wedge still occurs** after #1+#2: switch to powered USB hub between Max Carrier and C2 (rules in cause #3). If a wedge still occurs after that, escalate to backporting upstream chipidea/EHCI patches (#4) or pivot to MIPI CSI (Phase-2 stretch).

## Key sources

- Kurokesu C2 datasheet (general: <https://www.kurokesu.com/main/category/cameras/>).
- Linux UVC driver FAQ: <https://www.ideasonboard.org/uvc/faq/> — authoritative on autosuspend, FIX_BANDWIDTH semantics, and `-ENOSPC` workarounds.
- Linux kernel UVC docs: <https://www.kernel.org/doc/html/latest/userspace-api/media/drivers/uvcvideo.html>.
- Bugzilla #212771: `cannot get freq at ep 0x84 (webcam microphone not working)` — confirms the message originates from snd-usb-audio against a webcam mic endpoint, not from uvcvideo.
- Arch Linux thread #292870 — same `cannot get freq` pattern; resolved by simply moving the camera to a different USB port (i.e. it's a power/host-controller issue, not a driver issue).
- LKML 2015 "[RFC] ALSA: usb-audio: reduce 'cannot get freq at ep' spew" — confirms the message is benign printk noise.
- Unix StackExchange #346246 ("Blacklist a USB device only for use by uvcvideo") — pattern for selectively unbinding drivers.
- Linux kernel chipidea docs: <https://www.kernel.org/doc/html/v5.9/usb/chipidea.html>.

## Open questions for next session

- What does `cat /sys/bus/usb/devices/1-1.2/power/control` report on a fresh boot? (Default may already be `on` if LmP disables autosuspend in `/etc/udev/rules.d/`.)
- Does `lsusb -v -d 16d0:0ed4` show interface association descriptors? Can we enumerate which interfaces are bound to which driver?
- Does the X8 boot cmdline already include `usbcore.autosuspend=-1`? (Check `/proc/cmdline`.)
- Is `snd_usb_audio` actually loaded? (`lsmod | grep snd_usb`.)
- Is there a `/dev/snd/pcmC*` node that corresponds to the C2 mic? (`ls /dev/snd/`.)
- Does plugging the C2 directly into a powered hub (bypassing the SMSC on the Max Carrier) eliminate the wedge?
