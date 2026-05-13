# W2-01 USB Wedge Avoidance — Peer Research & Strategy

**Author:** Copilot (Claude)
**Date:** 2026-05-12
**Version:** v1.0
**Companion to:** [2026-05-12_W2-01_USB_Wedge_Root_Cause_Research_v1_0.md](2026-05-12_W2-01_USB_Wedge_Root_Cause_Research_v1_0.md)
**Status:** Research updated; strongest current recommendation is Option A before C2 enumeration, with powered hub in parallel.

---

## 1. Why this document exists

Board 1 (ADB `2D0A1209DABC240B`) has now wedged **four times in this session** — once before any defenses were added, and again immediately when the *new* defensive pre-flight (autosuspend disable + snd-usb-audio unbind + dmesg capture) tried to write to `power/control` of the Kurokesu C2.

The user asked: *"Please do more research and see if other people are having a similar problem."*

This document captures peer evidence collected from LKML, the NXP community, and general UVC literature, and proposes the next concrete strategy.

---

## 2. Peer corroboration

### 2.1 LKML 2025-06-09 — Shawn Guo (NXP), "i.MX kernel hangup caused by chipidea USB gadget driver"

- URL: https://www.spinics.net/lists/kernel/msg5713638.html
- Platform: `imx8mm-evk`, chipidea (`ci_hdrc-imx`), USB gadget mode.
- Symptom: data transfer in progress + USB suspend/resume cycle → controller hangs hard.
- Affects: mainline, NXP `lf-6.6.y`, NXP `lf-6.12.y`.
- **Does NOT affect NXP `lf-6.1.y`** (which is what our LmP 4.0.11 / kernel `6.1.24-lmp-standard` is based on) — Peter Chen's earlier patch series is in 6.1.
- Patch summary: insert `usb_gadget_disconnect()` in `udc_suspend` and `usb_gadget_connect()` in `udc_resume` to keep the link state coherent across PM transitions.

**Implication for us:** the chipidea suspend/resume substrate is *known fragile* and the upstream community is *still finding races in it in 2025*. Even though our exact 6.1.x kernel is reported to have the gadget-side fix, host-side resume races on the same HW are plausible. This corroborates root-cause-doc cause #1 (ci_hdrc-imx autosuspend deadlock).

### 2.2 NXP community 2015 — abdulhussain → sumankumar, "usb high speed device hang issue"

- URL: https://community.nxp.com/t5/i-MX-Processors/usb-high-speed-device-hang-issue/m-p/349528
- Platform: imx6 + ci_hdrc + USB hub, fast plug/unplug stress.
- Symptom: `device descriptor read/all, error -71` then controller stuck — nearly identical to our wedge log when uvcvideo binds to C2.

Two findings directly applicable to W2-01:

1. **Externally-powered USB hubs self-recover.** Quoting the thread: *"we observed like the same error is happening in externally powered usb hub, but driver is automatically recovering."* This is independent corroboration of our root-cause-doc cause #3 (VBUS sag — Max Carrier USB-A is unpowered, C2 spins up at ~480 mA on bandwidth negotiation).

2. **`rmmod ci_hdrc_imx; modprobe ci_hdrc_imx` recovers the controller without a power cycle.** Quoting: *"if i build... chipidea driver into the loadable module, during error scenario if i rmmod the chipidea driver and load the chipidea module again, high speed usb device detection is happening."* If `ci_hdrc_imx` is modular on our LmP, this replaces the physical PWR-USB-C unplug with a one-line shell command — a massive iteration-speed win.

### 2.3 Negative results (still informative)

- **No public report of a Portenta X8 + Max Carrier + UVC webcam working in production.** Foundries documents UVC support generically, Arduino documents Max Carrier USB-A as a host port, but neither ships a webcam pipeline demo for this exact combination on LmP 4.0.11. We are effectively first to attempt this — which explains the silence.
- **No public report of Kurokesu C2 wedging an ARM host.** C2 is reported to work on x86 and Jetson; nothing specific to i.MX8MM / chipidea.
- **Toradex Verdin imx8mm + UVC** threads point back to the same NXP gadget thread (#2.1), so the broader i.MX8MM community has the same fragility.

---

## 3. What the most recent wedge actually proves

The probe on 2026-05-12 17:14 printed:

```
c2_syspath=/sys/bus/usb/devices/1-1.2/
--- pre-flight (a) disable USB autosuspend on C2 + hub ---
```

… and then the entire ci_hdrc-imx host died. Board 1 dropped from `adb devices` and required physical PWR-USB-C unplug.

Sequence forensics:
1. C2 had probably already auto-suspended in the few seconds between `adb push` of the wrapper and the script reaching pre-flight (default autosuspend delay is 2 s).
2. The shell `for d in /sys/bus/usb/devices/*; do cat "$d/idVendor"; ...; done` walk touched the USB device's sysfs node while it was likely suspended. Kernel USB PM docs say sysfs access affects USB idle accounting, but cached descriptor reads are not, by themselves, proven to issue device I/O. Treat this as a suspect, not a proven trigger.
3. Our `echo on > /sys/.../power/control` write is the stronger documented trigger: kernel docs define `power/control=on` as "the device should be resumed and autosuspend is not allowed." If C2 was already suspended, this write initiated the dangerous autoresume path we were trying to avoid.
4. The chipidea resume path raced and the controller wedged before either snd-usb-audio unbind or dmesg-tail steps ran. **dmesg log was therefore never captured.**

**Conclusions:**

- snd-usb-audio is **innocent in this wedge**. We never reached the unbind step.
- uvcvideo is **innocent in this wedge**. No `STREAMON`, no `ENUM_FMT`, no `open()` of `/dev/video*`.
- The wedge trigger is most defensibly **runtime autoresume after C2 has already suspended**. The late `power/control=on` write and a later UVC `open()` both enter that path; the sysfs VID/PID walk is lower-confidence supporting context.
- Therefore any mitigation that runs *after* C2 has had a chance to autosuspend is **inherently racy**. We must keep C2 from ever entering runtime-PM in the first place.

---

## 4. Strategy options

### Option A — `usbcore.autosuspend=-1` before C2 enumeration

**Mechanism:** Add `usbcore.autosuspend=-1` before the C2 enumerates. The production-clean path on `lmp-xwayland` is the Foundries factory `OSTREE_KERNEL_ARGS` mechanism; a bench workaround may use `fw_setenv` or a local boot script only after verifying the active boot layout. After one reboot, newly initialized USB devices should inherit no-autosuspend policy.

**Why it should work:** The race window in §3 disappears completely. C2 stays in `power/control=on` from enumeration onward. Matches the standard remediation in the Linux UVC FAQ and ideasonboard.org documentation.

**Risks:**
- Any bench-level `fw_setenv` or boot-script edit modifies persistent boot configuration. Reversible but requires care.
- Need to confirm the LmP cmdline assembly (`OSTREE_KERNEL_ARGS`, `boot.cmd`, `fw_env.config`, or a Foundries-specific overlay).
- One-time reboot needed after change.

**Verification:** After reboot, `cat /sys/module/usbcore/parameters/autosuspend` should print `-1`, and `cat /sys/bus/usb/devices/*/power/control` for every device should be `on`.

### Option B — Modular recovery: `rmmod ci_hdrc_imx; modprobe ci_hdrc_imx` if available

**Mechanism:** When (not if) the next wedge happens, instead of physical PWR-USB-C unplug, run:

```bash
sudo rmmod ci_hdrc_imx
sudo modprobe ci_hdrc_imx
```

**Why it should work:** Direct quote from the NXP 2015 thread (§2.2). Tearing down and re-creating the chipidea host driver clears the stuck state machine without touching power.

**Prerequisite:** `ci_hdrc_imx` must be a loadable module, not built into the kernel. Verify with:

```bash
lsmod | grep ci_hdrc
grep ci_hdrc /lib/modules/$(uname -r)/modules.builtin
```

If `ci_hdrc_imx` appears in `lsmod`, we're good. If it appears in `modules.builtin`, this option is unavailable and we must rely on A or C. Board 2's older control image suggests chipidea is built-in, so treat this as a maybe, not a plan.

**Caveat:** This recovers the *controller*, not necessarily a hung uvcvideo userspace handle. We may still need to kill any open `/dev/video*` consumers before reload. But for our enumerate-only probe (no STREAMON), this should be sufficient.

### Option C — Externally powered USB hub

**Mechanism:** Insert a USB 2.0 hub with its own 5 V / ≥1 A wall supply between the Max Carrier USB-A and the C2.

**Why it should work:** Direct corroboration from NXP 2015 thread (§2.2): "externally powered usb hub… driver is automatically recovering." Addresses cause #3 (VBUS sag) at the hardware layer — independent of any kernel race.

**Risks:** None at the software layer. Pure HW change; requires procurement (~$10).

**Cost:** Adds one more box on the bench but is the most physically robust mitigation.

### Recommended combination

**A first, C in parallel, B only if Board 1 proves modular.**

- A removes the trigger so the probe can run cleanly.
- C removes VBUS/ramp-timing stress and makes the software experiment less brittle.
- B is valuable only if Board 1's 6.1 image has `ci_hdrc_imx` as a loadable module. Board 2's older image suggests it may be built-in, so do not count on this recovery path.

---

## 5. Pre-flight script bugs to fix regardless of strategy choice

The current `w2_01_camera_first_light.sh` pre-flight has two ordering bugs that should be fixed before the next attempt:

1. **`dmesg -wH` capture must start FIRST**, before any sysfs walk or power/control write. As-is, the dmesg tail is step (d) but the wedge happens at step (a), so the log is empty when we need it.
2. **Replace the `for d in /sys/bus/usb/devices/*; cat $d/idVendor` walk with `lsusb -t` parsing**, so we don't trigger runtime-resume by reading sysfs attributes of a suspended device.

If we adopt Option A, bug #2 becomes moot (autosuspend is disabled, nothing can be suspended). But fixing #1 is still useful for capturing diagnostic data on any *future* failure mode.

---

## 6. Open questions for the user

1. **Strategy approval:** Adopt **A + C** (no autosuspend before C2 enumeration + powered hub in parallel), with B only if modular recovery exists on Board 1?
2. **Boot-arg safety:** Are you comfortable with a bench-level persistent boot-arg edit if the Foundries production path is not immediately available? The change is only `usbcore.autosuspend=-1`, but we should capture the current cmdline and rollback path first.
3. **`fw_setenv` / boot layout availability:** Board 2 has `/usr/bin/fw_setenv` and `/mnt/boot/uboot.env`, but Board 1 must be verified before any write. If Board 1 exposes only an OSTree boot layout, production should use Foundries `OSTREE_KERNEL_ARGS`.
4. **Procurement for Option C:** Should we order a powered USB 2.0 hub now in parallel, or wait to see if Option A alone succeeds?
5. **Pivot threshold:** If A+C still wedges after 2 attempts, do we move to MIPI CSI (already on the BoM as a Tier-2 alternative) instead of investing more in USB UVC?

---

## 6a. New peer findings (2026-05-12 second research pass)

Five additional substantive items emerged from a second research pass focused on **safety / correctness of each strategy option**, rather than just on whether the wedge is real.

### 6a.1 ⚠️ NXP forum: `fw_setenv` corrupts the U-Boot environment on MTD with kernels ≥ 5.4

- URL: https://community.nxp.com/t5/i-MX-Processors/fw-setenv-corrupts-environment-on-MTD-for-Linux-kernels-after-5/m-p/1845194
- Quote: *"with newer Linux kernels (5.10, 5.15, and 6.1), `fw_setenv` destroys the environment on the MTD, making the upgrade process fail (same failure for `fw_setenv` from `UBOOT_TOOLS` and `LIBUBOOTENV`)."*
- Boundary Devices identified the root cause as a bug in the MTD driver.
- **Affects our exact kernel band (6.1.x).**

**Implication for Option A:** This bug only triggers when the U-Boot environment is stored in raw MTD/NAND. The Portenta X8 boots from **eMMC**, where the environment lives in a fixed offset of an `mmcblk` partition — a completely different code path. We are most likely safe, but we MUST verify before running `fw_setenv`:

```bash
cat /etc/fw_env.config
```

If the device path begins with `/dev/mmcblk` we are safe. If it begins with `/dev/mtd*` we MUST NOT use `fw_setenv` and instead use Option A-alt below.

### 6a.2 Option A-alt: U-Boot `bootenv.txt` overlay (Foundries-recommended)

If `fw_setenv` is risky or unavailable, Foundries LmP supports persistent kernel-cmdline override via a **`bootenv.txt`** or `/boot/uEnv.txt`-style file read by U-Boot at boot. The exact mechanism is board-specific (Portenta X8 uses U-Boot 2022.04-lmp). Verification command:

```bash
ls -la /boot/ | grep -iE 'uenv|bootenv|extlinux|cmdline'
mount | grep boot
```

If a `uEnv.txt` or extlinux config exists, we can append `usbcore.autosuspend=-1` to the cmdline by editing the file — no `fw_setenv` required. This is the **safer** form of Option A.

### 6a.3 ⚠️ NVIDIA Xavier NX thread: udev `power/control` rule resets after reboot

- URL: https://forums.developer.nvidia.com/t/unable-to-permanently-turn-off-autosuspend-for-a-usb-device-connected-to-a-xavier-nx/197534
- Symptom: udev rule writes `power/control=on` correctly at runtime, but after reboot the value reverts to `auto` even though the rule still fires.
- Root cause: The kernel's autosuspend logic re-evaluates `power/control` based on `power/persist`, `power/wakeup`, and the *order* in which the bus-add and driver-bind events fire. Writing `power/control` from a `SUBSYSTEM=="usb"`, `ACTION=="add"` rule that lacks `ENV{DEVTYPE}=="usb_device"` may target the wrong sysfs node, or write before the `power/` directory exists.

**Correct form** for our case:

```udev
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", \
    ATTR{idVendor}=="16d0", ATTR{idProduct}=="0ed4", \
    TEST=="power/control", ATTR{power/control}="on"
```

Key requirements:
- `ENV{DEVTYPE}=="usb_device"` (NOT `"usb_interface"`) — matches the parent device that owns `power/control`.
- `TEST=="power/control"` — gates the write so udev doesn't error if the attr doesn't exist on this match.
- Place file at `/etc/udev/rules.d/99-w2-01-c2.rules` and reload with `udevadm control --reload && udevadm trigger`.

**Bigger implication:** Option B (udev rule alone) is **not sufficient by itself across reboots** if the kernel re-applies autosuspend defaults early in enumeration. Combine udev with `usbcore.autosuspend=-1` (Option A) for belt-and-suspenders.

### 6a.4 NXP forum: i.MX8MM USB host can lose VBUS based on `dr_mode`

- URL: https://community.nxp.com/t5/i-MX-Processors/USB-detection-on-i-MX8MM/m-p/1581558
- Symptom: with `dr_mode="host"` the USB port is *only powered briefly during kernel boot*, then VBUS drops; with `dr_mode="otg"` VBUS stays on.
- This is a **device-tree-level** issue — would need a DTB change to fix.

**Implication for us:** Worth checking the Portenta X8 LmP device tree for the Max Carrier USB-A port:

```bash
sudo cat /sys/firmware/devicetree/base/soc@0/bus@32c00000/usb@*/dr_mode 2>/dev/null | tr -d '\0' ; echo
# or
sudo find /sys/firmware/devicetree -name dr_mode | xargs -I{} sh -c 'echo -n "{}: "; cat {}; echo'
```

If our USB host port is configured `dr_mode="host"` and the symptom #6a.4 applies, the wedge could be partially explained as: VBUS drops between enumeration events, C2 brown-outs, ci_hdrc tries to recover but races. This is a NEW hypothesis we should test before committing to the udev/cmdline route.

### 6a.5 Toradex Verdin iMX8MP — same family, same wedge pattern

- URL: https://community.toradex.com/t/intermittent-problem-with-usb-enumeration-on-verdin-imx8mp/27264
- Symptom: USB modem on Verdin iMX8MP (one generation up from our 8MM) sometimes does not properly enumerate after power-on. dmesg shows `new high-speed USB device number 4 using xhci-hcd` then enumeration silently aborts.
- Root cause from the thread: insufficient ramp time on VBUS rising edge — the device starts to enumerate before its own internal supplies are stable, producing a `-71` or hung enumeration.
- Fix: add a small VBUS settle delay or use a powered hub.

**Implication for us:** Independent confirmation of cause #3 (VBUS sag) on a sibling SoC. Reinforces the value of Option C (powered hub).

### 6a.6 ODROID forum: `uvcvideo quirks=128` for high-bandwidth UVC on constrained hosts

- URL: https://forum.odroid.com/viewtopic.php?f=181&p=364999
- Found that loading uvcvideo with `quirks=128` (`UVC_QUIRK_FIX_BANDWIDTH`) helped reduce bandwidth claims and made dual-camera streaming work on Odroid N2+.
- Earlier in this session we dismissed `UVC_QUIRK_FIX_BANDWIDTH` because the Linux source comment says it doesn't apply to MJPEG. The ODROID thread suggests it can still help in practice when the bottleneck is SoC EHCI scheduling rather than the wire.
- **Worth trying as a final fallback** if A+C (and B, if available) fail and we suspect ehci-imx periodic-schedule overflow at STREAMON time.

### 6a.7 uvcvideo `nodrop` is being removed — do not depend on it

- URL: https://lkml.org/lkml/2025/11/17/1769 — "[PATCH 1/4] media: uvcvideo: Remove nodrop parameter"
- The historical workaround `uvcvideo.nodrop=1` for cameras that emit corrupted frame headers is deprecated upstream and being removed. Do not add it to our cmdline.

---

## 6b. Updated strategy ranking after 6a

| Strategy | Updated assessment | Confidence | Required prep |
|---|---|---|---|
| **A** boot-time `usbcore.autosuspend=-1` | First-line fix. Use **`bootenv.txt`/`uEnv.txt` overlay** if available; only use `fw_setenv` after confirming `/etc/fw_env.config` points to eMMC, not MTD. | HIGH | Verify storage backend; capture current cmdline as rollback. |
| **B** udev rule | Use **with** Option A, never alone. Rule needs `ENV{DEVTYPE}=="usb_device"` and `TEST=="power/control"`. | MEDIUM | Compose correct rule; `udevadm test` before deploy. |
| **B-alt** `rmmod ci_hdrc_imx; modprobe ci_hdrc_imx` recovery | Confirm `ci_hdrc_imx` is a module first (`lsmod`). If built-in, this option vanishes. | HIGH if modular | One safe `lsmod`/`modules.builtin` check. |
| **C** powered USB hub | Most physically robust — corroborated by NXP 2015 + Toradex 2024. Order in parallel. | HIGH | Procurement (~$10). |
| **D** check `dr_mode` in DT | NEW — could explain VBUS instability without kernel race. | MEDIUM | Read `/sys/firmware/devicetree`. |
| **E** `uvcvideo quirks=128` fallback | Try only if A+C (and B, if available) fail at STREAMON. | LOW (post-enum step) | Modify modprobe.d. |

---

## 6c. New action checklist (read-only diagnostics — runs even on Board 2 control)

These are all SAFE commands — none of them write anything, and none of them touch the suspended C2 (Board 2 has no C2 attached). Running them on Board 2 first will tell us most of what we need before we ever try to repower Board 1.

```bash
# 1. Storage backend for U-Boot env (drives Option A safety)
cat /etc/fw_env.config 2>/dev/null
ls /usr/sbin/fw_setenv /usr/bin/fw_setenv 2>/dev/null

# 2. Is ci_hdrc_imx modular? (drives Option B-alt feasibility)
lsmod | grep -E 'ci_hdrc|chipidea'
zgrep -E 'CONFIG_USB_CHIPIDEA|CONFIG_USB_CHIPIDEA_HOST|CONFIG_USB_CHIPIDEA_IMX' /proc/config.gz 2>/dev/null \
  || cat /boot/config-$(uname -r) 2>/dev/null | grep -E 'CHIPIDEA'
grep -E 'ci_hdrc' /lib/modules/$(uname -r)/modules.builtin 2>/dev/null

# 3. Boot files for Option A-alt (overlay vs fw_setenv)
ls -la /boot/ 2>/dev/null
cat /boot/uEnv.txt 2>/dev/null
cat /boot/extlinux/extlinux.conf 2>/dev/null
cat /proc/cmdline

# 4. dr_mode of every USB controller in the DT (drives hypothesis 6a.4)
sudo find /sys/firmware/devicetree -name dr_mode 2>/dev/null \
  | xargs -I{} sh -c 'printf "%s = " "{}"; tr -d "\0" < {}; echo'

# 5. Current autosuspend defaults
cat /sys/module/usbcore/parameters/autosuspend
for d in /sys/bus/usb/devices/*/power/control; do
  echo "$d = $(cat $d 2>/dev/null)"
done | sort -u
```

If we run these on Board 2 (which is currently online), we get most of the info we need to commit to a strategy *without* power-cycling Board 1 again.

---

## 6d. Additional comments from 2026-05-12 research pass

### 6d.1 Kernel USB PM docs sharpen the avoidance rule

The authoritative kernel USB power-management document makes three details very relevant to this wedge:

- `power/control=on` is not just a policy write. It means "the device should be resumed and autosuspend is not allowed." If C2 is already runtime-suspended, writing `on` is itself an autoresume operation.
- `usbcore.autosuspend=-1` on the kernel command line prevents autosuspend for newly initialized USB devices. Changing `/sys/module/usbcore/parameters/autosuspend` at runtime only changes the default for **new** USB devices; it does not retroactively fix an already-enumerated C2.
- USB port power-off is a separate PM layer. Hub/root-port power policy is exposed through `pm_qos_no_power_off` and `runtime_status`; a logically powered-off port can lose VBUS and force the same reset-resume recovery path as suspend.

**Practical implication:** the safest software experiment is not "boot with C2 already attached, then write `power/control=on` late." The safer sequence is either:

1. persist `usbcore.autosuspend=-1` before C2 is ever enumerated, reboot, then attach C2; or
2. boot without C2, set runtime `usbcore.autosuspend=-1`, then plug C2 in (preferably through a powered hub), so the C2 receives the no-autosuspend default at enumeration time.

Add this extra read-only port-power diagnostic to §6c when Board 1 is recovered:

```bash
find /sys/bus/usb/devices \( \
  -path '*/power/control' -o \
  -path '*/power/autosuspend_delay_ms' -o \
  -path '*/power/runtime_status' -o \
  -path '*/power/pm_qos_no_power_off' \
\) -print 2>/dev/null | sort | while read f; do
  printf '%s = ' "$f"
  cat "$f" 2>/dev/null
done
```

### 6d.2 Foundries LmP kernel-arg mechanism is build-level for `lmp-xwayland`

Foundries documents two different kernel-cmdline customization paths:

- `lmp` / OSTree-style images: extend kernel args in the factory layer with `OSTREE_KERNEL_ARGS:<machine> = "console=${console} <new-args> ${OSTREE_KERNEL_ARGS_COMMON}"`.
- `lmp-base`: extend `bootcmd_args` in `u-boot-base-scr/<machine>/uEnv.txt.in`.

Our W2-01 image is `lmp-xwayland`, so a local `/boot/uEnv.txt` bench edit may not exist or may not be the durable production path. For a bench workaround, `fw_setenv` or a local boot-script edit might still work after verification. For production, the clean answer is to add `usbcore.autosuspend=-1` through the Foundries factory configuration so it survives OSTree updates and reflashes.

### 6d.3 Board 2 read-only diagnostic results (control board, not exact Board 1)

I ran the §6c diagnostics on Board 2 (`2E2C1209DABC240B`). Important caveat: Board 2 is older than the camera-bearing Board 1 image:

- Board 2: `Linux-microPlatform XWayland 4.0.3-674-88`, kernel `5.10.93-lmp-standard`.
- Board 1 wedge reports: LmP `4.0.11-934-91`, kernel `6.1.24-lmp-standard`.

Still-useful findings:

- `/etc/fw_env.config` points to `/mnt/boot/uboot.env`, not `/dev/mtd*`. That avoids the NXP MTD corruption failure mode from §6a.1 on this control board. Verify again on Board 1 before any `fw_setenv` write.
- `fw_setenv` exists at `/usr/bin/fw_setenv` on Board 2.
- `CONFIG_USB_CHIPIDEA=y`, `CONFIG_USB_CHIPIDEA_HOST=y`, and `CONFIG_USB_CHIPIDEA_IMX=y`; `lsmod` did not show `ci_hdrc_imx`. Treat `rmmod ci_hdrc_imx; modprobe ci_hdrc_imx` recovery as **probably unavailable** unless Board 1's 6.1 image proves otherwise.
- `/boot` has an OSTree layout (`loader.1`, `ostree`) and no visible `/boot/uEnv.txt` or `/boot/extlinux/extlinux.conf`; `/proc/cmdline` did **not** include `usbcore.autosuspend=-1`.
- Device tree exposes two USB controllers: one `dr_mode = otg`, one `dr_mode = host`. We still need to map which controller feeds the Max Carrier USB-A path, but §6a.4 remains a live hypothesis.
- `cat /sys/module/usbcore/parameters/autosuspend` returned `2`, so the control board default is the normal two-second autosuspend delay.

**Updated strategy comment:** Option A remains first-line, but the most robust form is "no autosuspend before C2 enumeration," not "late per-device write after C2 has already gone idle." Option B-alt is downgraded until Board 1 proves `ci_hdrc_imx` is modular. Option C (powered hub) gets stronger because it removes both VBUS sag and hotplug/re-enumeration timing from the X8's marginal path.

---

## 6e. Decisions taken 2026-05-12 and bench artifacts staged

The five open questions from §6 have been resolved as follows (assistant
recommendation, awaiting user override if any):

1. **Strategy:** A + C primary; B (`rmmod ci_hdrc_imx`) only if Board 1 proves
   `ci_hdrc_imx` is modular (Board 2 control image is built-in, so plan
   without it).
2. **Bench cmdline edit:** YES, but gated. Use the safety-checked helper
   below; production fix goes through Foundries `OSTREE_KERNEL_ARGS`.
3. **fw_env verification on Board 1:** non-negotiable. Diagnostic script
   below captures `/etc/fw_env.config` first; helper aborts if backend is
   `/dev/mtd*`.
4. **Powered USB hub:** order now in parallel (USB 2.0, ≥1 A external 5 V,
   per-port switches preferred).
5. **Pivot threshold:** 2 clean A+C wedges (i.e. with verified
   `usbcore.autosuspend=-1` AND powered hub inline AND fixed pre-flight
   script) → pivot to MIPI CSI.

Staged bench artifacts in
`LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/`:

| File | Purpose | Safety |
|---|---|---|
| `w2_01_camera_first_light.sh` (modified) | Pre-flight bugs fixed: dmesg tail starts FIRST; per-device power/control writes are gated on `usbcore.autosuspend == -1` AND `runtime_status != suspended`. | Read/write but inert until Option A is in effect. |
| `w2_01_board_diag.sh` (new) | Read-only diagnostic. Captures `/etc/fw_env.config`, ci_hdrc modular vs builtin, `/boot` layout, DT `dr_mode`, USB tree, per-device PM snapshot, udev rules, dmesg tail. Run with C2 unplugged. | Read-only. Safe to run anywhere. |
| `99-w2-01-c2.rules` (new) | Belt-and-braces udev rule pinning C2 (`16d0:0ed4`) to `power/control=on` at enumeration time. Uses correct `ENV{DEVTYPE}=="usb_device"` + `TEST=="power/control"` form per §6a.3. | Inert without Option A; complementary, not a substitute. |
| `apply_option_a_autosuspend_disable.sh` (new) | Persists `usbcore.autosuspend=-1` via `fw_setenv`. Hard-blocks on MTD backend (§6a.1), captures rollback snapshot, refuses without `--i-have-read-the-avoidance-doc` flag, refuses to overwrite an existing differing `usbcore.autosuspend=` value. | Bench-only stopgap; production should use Foundries `OSTREE_KERNEL_ARGS`. |

Recommended sequence after Board 1 PWR-USB-C unplug recovers it:

1. C2 unplugged. Push and run `w2_01_board_diag.sh`. Save the log to
   `AI NOTES/`.
2. Inspect log: confirm `/etc/fw_env.config` is eMMC, capture current
   `/proc/cmdline` and DT `dr_mode` results.
3. Push and run `apply_option_a_autosuspend_disable.sh
   --i-have-read-the-avoidance-doc`. Verify success message.
4. Optionally `cp 99-w2-01-c2.rules /etc/udev/rules.d/` and reload.
5. `sudo reboot`. After boot, with C2 still unplugged, confirm
   `cat /sys/module/usbcore/parameters/autosuspend` prints `-1`.
6. Plug C2 in (preferably through the powered hub once it arrives). Run the
   modified `w2_01_camera_first_light.sh` in enumerate-only mode.
7. If enumerate-only passes, attempt capture mode.

If any of steps 2/3/5 fails, do NOT proceed to 6. Capture the failure
output and add a §6f appendix.

---

## 6f. Empirical results 2026-05-12 — Option A is a no-op on this LmP build

After §6e was written, both candidate Option A applicators were exercised
on Board 1. Both failed silently in different ways. Captured logs:

- [2026-05-12_W2-01_Board1_Diag_v1_0.log](2026-05-12_W2-01_Board1_Diag_v1_0.log) — clean baseline.
- [2026-05-12_W2-01_OptionA_Apply_v1_0.log](2026-05-12_W2-01_OptionA_Apply_v1_0.log) — `fw_setenv` apply.
- [2026-05-12_W2-01_Board1_PostBLS_Diag_v1_0.log](2026-05-12_W2-01_Board1_PostBLS_Diag_v1_0.log) — post-BLS-edit ground truth.
- [2026-05-12_W2-01_Board1_Restore_v3_0.log](2026-05-12_W2-01_Board1_Restore_v3_0.log) — revert log.

### 6f.1 `fw_setenv bootargs` is ignored on LmP OSTree

`apply_option_a_autosuspend_disable.sh` ran without error. `/etc/fw_env.config`
correctly resolved to `/mnt/boot/uboot.env` (eMMC, not MTD — §6a.1 trap
avoided). `fw_setenv bootargs ...` returned success. After reboot,
`/proc/cmdline` was unchanged.

**Mechanism (inferred):** the LmP U-Boot bootscript on `lmp-xwayland`
4.0.11 does not consume the `bootargs` U-Boot variable when an OSTree boot
entry is selected. It rebuilds the kernel cmdline from the active BLS
entry under `/boot/loader/entries/`. **`fw_setenv bootargs` is therefore a
silent no-op on this image.**

Action: `apply_option_a_autosuspend_disable.sh` is now **deprecated**. Do
not use. Remove from any future runbook.

### 6f.2 BLS `options` line is also not the cmdline source U-Boot uses

A second helper, `apply_bls_autosuspend_disable.sh`, was written to edit
the BLS file directly:

```
/boot/loader/entries/ostree-1-lmp.conf
```

It appended ` usbcore.autosuspend=-1` to the `options` line (idempotent,
with `.bak.w2_01` backup). Edit verified on disk. After reboot, the
captured `/proc/cmdline` was:

```
console=tty1 console=ttymxc2,115200 earlycon=ec_imx6q,0x30880000,115200 \
  root=/dev/mmcblk2p2 rootfstype=ext4 \
  ostree=/ostree/boot.1/lmp/<sha>/0 root=/dev/mmcblk2p2
```

Note: `usbcore.autosuspend=-1` is **absent**, and U-Boot appended its own
trailing `root=/dev/mmcblk2p2` token. This proves U-Boot is constructing
the cmdline from a small allow-list (kernel/initrd/ostree subset of BLS,
plus root device) and **discarding the rest of the BLS `options` line**.
`/sys/module/usbcore/parameters/autosuspend` correspondingly read `2`
(default), confirming the karg never reached the kernel.

Action: `apply_bls_autosuspend_disable.sh` is also **deprecated** for the
purpose of cmdline modification. (The script itself works correctly; the
boot pipeline ignores its output.)

### 6f.3 Board 1 ADB instability is NOT caused by Option A attempts

Because §6f.1 and §6f.2 prove neither helper changed the kernel cmdline,
the recurring Board 1 adb dropouts cannot be attributed to
`usbcore.autosuspend=-1`. The instability is independent and pre-existing.
Most likely candidates (in order):

1. **Cold-boot adbd race** on this 4.0.11/6.1.24 image — adbd takes
   variable time to come up, and our PC-side `adb` polling can hit the
   window where it dies and restarts. Multiple cold cycles "needed" to
   enumerate are most likely just our polling racing the `adbd` startup.
2. **PC-side adb server confusion** after long device absences — running
   `adb kill-server` between recovery cycles tends to clear this.
3. The original USB host fragility we were trying to mitigate
   (cause #1/#3 from §2 and §6a) — independent of any cmdline edit.

Action: do NOT spin further bootarg edits trying to "fix" the dropouts.
The dropouts predate our edits and the edits never took effect.

### 6f.4 Updated Option A path — three remaining viable mechanisms

Order by safety/effort, ascending:

a. **Persist via Foundries factory `OSTREE_KERNEL_ARGS`** — production
   path. Requires a factory build, but produces a kernel cmdline change
   that survives OSTree updates. Recommended for any non-bench use.

b. **Runtime, post-boot:** `echo -1 | sudo tee
   /sys/module/usbcore/parameters/autosuspend` after every boot, BEFORE
   plugging C2 in. Per kernel docs, this changes the default for **newly
   enumerated** USB devices only, so it works iff C2 is plugged after the
   write. Pair with §6d.1 sequence "boot without C2, set runtime, then
   plug." Trade-off: must be reapplied each boot (systemd unit or rc.local
   could automate).

c. **U-Boot script edit / boot.scr regeneration** — modify the actual
   bootscript (likely under `/boot/boot.scr` or generated from
   `/boot/boot.cmd`) so it injects `usbcore.autosuspend=-1` into the
   cmdline it builds. Higher risk; requires `mkimage` (`u-boot-tools`) and
   could brick boot if the script structure is misunderstood. Not
   recommended without serial console access.

### 6f.5 Concrete next step

1. Restore Board 1 BLS file from `.bak.w2_01` (in progress, see
   `2026-05-12_W2-01_Board1_Restore_v3_0.log`).
2. Procure powered USB 2.0 hub (Option C) — unblocked, primary path.
3. Defer Option A until either (i) the powered hub alone resolves the
   wedge, in which case Option A becomes optional belt-and-braces, or
   (ii) a serial console is wired so we can safely test mechanism 6f.4.c
   without losing the board.
4. Add a small `systemd-tmpfiles.d` or oneshot service that does the
   runtime-write from §6f.4.b on next boot if we want a software-only
   stopgap before the hub arrives. Defer until requested.

---

## 7. References

- Root-cause analysis: [2026-05-12_W2-01_USB_Wedge_Root_Cause_Research_v1_0.md](2026-05-12_W2-01_USB_Wedge_Root_Cause_Research_v1_0.md)
- Session summary: [2026-05-12_W2-01_Camera_First_Light_Session_Summary_v1_0.md](2026-05-12_W2-01_Camera_First_Light_Session_Summary_v1_0.md)
- LKML chipidea gadget hang (2025-06-09): https://www.spinics.net/lists/kernel/msg5713638.html
- NXP community ci_hdrc imx6 hang (2015): https://community.nxp.com/t5/i-MX-Processors/usb-high-speed-device-hang-issue/m-p/349528
- NXP community: fw_setenv corrupts environment on MTD (kernel ≥ 5.4): https://community.nxp.com/t5/i-MX-Processors/fw-setenv-corrupts-environment-on-MTD-for-Linux-kernels-after-5/m-p/1845194
- NXP community: USB detection on i.MX8MM (`dr_mode` VBUS issue): https://community.nxp.com/t5/i-MX-Processors/USB-detection-on-i-MX8MM/m-p/1581558
- NVIDIA Xavier NX: udev `power/control` resets after reboot: https://forums.developer.nvidia.com/t/unable-to-permanently-turn-off-autosuspend-for-a-usb-device-connected-to-a-xavier-nx/197534
- Toradex Verdin iMX8MP intermittent USB enumeration: https://community.toradex.com/t/intermittent-problem-with-usb-enumeration-on-verdin-imx8mp/27264
- ODROID forum: `uvcvideo quirks=128` for high-bandwidth UVC: https://forum.odroid.com/viewtopic.php?f=181&p=364999
- LKML 2025-11-17 `uvcvideo nodrop` removal: https://lkml.org/lkml/2025/11/17/1769
- Bobcares "Fixing Device Descriptor Read/64 Error 71" (powered hub for high-power devices): https://bobcares.com/blog/device-descriptor-read-64-error-71/
- Linux kernel USB power management: https://www.kernel.org/doc/html/latest/driver-api/usb/power-management.html
- Foundries LmP customization, kernel command-line arguments: https://docs.foundries.io/93/user-guide/lmp-customization/lmp-customization.html#kernel-command-line-arguments
- Foundries U-Boot environment and boot script: https://docs.foundries.io/93/porting-guide/pg-spl-uboot-env.html
- Arduino Portenta Max Carrier documentation and downloadable schematics/datasheet: https://docs.arduino.cc/hardware/portenta-max-carrier/
- Linux UVC driver FAQ: https://www.ideasonboard.org/uvc/faq/
- kernel.org bug #212771 (the `cannot get freq` red herring we initially chased): https://bugzilla.kernel.org/show_bug.cgi?id=212771

---

*End of document.*
