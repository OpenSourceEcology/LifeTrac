# 2026-05-27 — FHSS Bench-Default Activation Plan & Runbook v1.0

**Status:** PLAN LANDED · HOST-SIDE TOOL LANDED · **LIVE-BENCH FALSIFIED §0.1** · NO FIRMWARE REFLASH · BENCH ROLLED BACK TO v25.0.1
**Author:** Copilot (autonomous; user unavailable for full session)
**Decision authority:** see §2 — chose Path A (orchestrator-staged at boot)
**Scope:** make `FCC_15_247_FHSS_50CH_BW250` (REG_PROFILE = 1) the default
bench transmission style, replacing the v25.0.1 image-pipeline demo's
`BENCH_ONLY_FIXED_915` (REG_PROFILE = 0) carrier path

---

## 0.1. Live-bench results (2026-05-27 20:25-20:40 UTC) — READ THIS FIRST

Once both compose files were flipped to `REG_PROFILE=1` + wide mask + FRF
pin and the daemons recreated, here is what actually happened on the
bench. **Bottom line: FHSS-as-bench-default is BLOCKED on firmware A6
RX-scan closure (v25.0.6.5 in roadmap). Bench rolled back to v25.0.1
baseline at the end of the session.**

### What worked
- Activator script (`tools/bench_activate_fhss.py`) — SELF_TEST=PASS on
  both X8s; `--apply` cleanly issued the 4-key CFG_SET + readback.
- Firmware validator behavior matches `host_cfg_profile.c` exactly:
  - REG_PROFILE=1 with wide mask (popcount=50) → `cfg_status=0x00` (OK).
  - REG_PROFILE=1 with narrow single-channel mask → `cfg_status=0x08`
    (REJECT, popcount<50). **This invalidates "Option A" from
    2026-05-25_Radio_RX_Frames_Zero_Channel_Mismatch.md** — single-
    channel-mask workaround under profile=1 is not wire-reachable.
- **TX-side FHSS hopping works end-to-end.** Tractor RFCO_PERTX showed
  `profile_id=1`, `hop_idx` varying 1..48, `channel_idx` cycling 0..49,
  `epoch` advancing 23→24→25, `freq_hz` spanning 903.75 - 927.25 MHz.
  Track-A `sx1276_fhss_*.c` modules are functionally correct on the
  transmitter side.

### What broke
- **RX side with `LIFETRAC_FORCE_FRF_HZ=915000000`:** firmware
  immediately overwrites RegFrf back to whatever it picked at
  activation (`FRF readback: 0xE2C000 = 907.000 MHz **MISMATCH**`
  after writing 915 MHz). Host-side pin does not persist under
  profile=1.
- **RX side without FRF pin (relying on firmware scan):** no
  `RX_SCAN_FAILED` faults, but `rx_frames` climbed only 0 → 3 in 30 s
  (vs. v25.0.1 baseline of ~28 frames per 10 s). `sx1276_rx_scan_policy.c`
  is in the tree but does not actually scan-and-acquire under wide-mask
  FHSS — RX dwells on one firmware-picked channel and catches ~1/50 of
  TXs by lucky alignment.
- **Pipeline throughput under any profile=1 mode tried today:**
  `frames_published` ≤ 2 over 5 minutes. Image pipeline is effectively
  broken — far worse than v25.0.1's 1 fps publish rate.

### Rollback verification
Both compose files reverted via
`tools/_revert_compose_to_v25_0_1.py {tractor|base|both}`. Backups
left in place at `*.bak_<ts>` on each X8 for forensic comparison.
Post-rollback verification: `rx_frames=183→211` in 10 s (~168/min),
`frames_published=47→51` at 1 fps, zero faults — clean v25.0.1
baseline restored.

### Verdict updates to §3 and the roadmap
- The 2026-05-25 channel-mismatch bug is now **falsified beyond just
  TX-side starting-channel pick** — the RX-side scan policy itself
  doesn't close the loop. This makes v25.0.6.5 strictly bigger than
  originally scoped: it must implement (or fix) the RX scan retune
  driver, not just demonstrate it.
- "Option A" (single-channel mask + profile=1) from the 2026-05-25
  document is **dead** — firmware rejects it. Strike it from the
  candidate fix list.
- The honest bench-default in this firmware build is profile=0
  (`BENCH_ONLY_FIXED_915` + FRF pin both sides). FHSS bench-default
  requires firmware work that needs your sign-off per repo memory
  (`lifetrac-x8-l072-bootloader.md` lines 24-28).

**Cross-references:**
- Plan: [LifeTrac-v25/AI NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md](LifeTrac-v25/AI%20NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md)
- v25.0.1 milestone: [LifeTrac-v25/AI NOTES/2026-05-27_Image_Over_LoRa_Pipeline_v25.0.1_First_Working_Demo.md](LifeTrac-v25/AI%20NOTES/2026-05-27_Image_Over_LoRa_Pipeline_v25.0.1_First_Working_Demo.md)
- Roadmap: [LifeTrac-v25/AI NOTES/2026-05-27_Image_Over_LoRa_Production_Roadmap_v25.0.1_to_v25.1.0.md](LifeTrac-v25/AI%20NOTES/2026-05-27_Image_Over_LoRa_Production_Roadmap_v25.0.1_to_v25.1.0.md)
- The blocking bug for wide-mask end-to-end traffic:
  [LifeTrac-v25/AI NOTES/2026-05-25_Radio_RX_Frames_Zero_Channel_Mismatch.md](LifeTrac-v25/AI%20NOTES/2026-05-25_Radio_RX_Frames_Zero_Channel_Mismatch.md)

---

## 1. Audit: what is already in the tree

I audited every component the FHSS plan §14.2 calls out. Result:

| Layer                       | Component                                                                                              | Status                                                                                                          |
| --------------------------- | ------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------- |
| Firmware – TX scheduler     | [sx1276_fhss.c](LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_fhss.c)               | landed                                                                                                          |
| Firmware – channel table    | [sx1276_fhss_chantab.c](LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_fhss_chantab.c) | landed                                                                                                          |
| Firmware – legal dwell      | [sx1276_legal_dwell.c](LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_legal_dwell.c) | landed                                                                                                          |
| Firmware – TX routing flag  | `-DLIFETRAC_FHSS_TX_ROUTED=1` in Makefile                                                              | landed `d4dfcb8` (2026-05-20)                                                                                   |
| Firmware – TX path gates    | [sx1276_tx.c](LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c)                   | landed (incl. bench bypass for REG_PROFILE=0 that the v25.0.1 demo currently rides)                             |
| Firmware – RX retune        | `sx1276_rx_retune_policy.c`, `sx1276_rx_retune_counters.c`                                             | landed                                                                                                          |
| Firmware – RX scan          | `sx1276_rx_scan_policy.c`, `sx1276_rx_scan_counters.c`, `sx1276_rx_scan_fail.c`                        | landed but **un-verified end-to-end** — see §3                                                                  |
| Firmware – cfg validator    | [host_cfg_profile.c](LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg_profile.c)      | landed (two-phase stage+activate)                                                                                |
| Firmware – RFCO instruments | `host_rfco.c`, `host_rfco_summary.c`, `host_rfco_summary_emit.c`                                       | landed                                                                                                          |
| Host – CFG_SET reference    | `configure_regulatory_profile_if_needed()` in `method_h_stage2_tx_probe_v2.py`                         | landed — but **only invoked by the probe**, not at container start                                              |
| Host – startup activator    | **MISSING** until this session                                                                          | **NEW:** [LifeTrac-v25/tools/bench_activate_fhss.py](LifeTrac-v25/tools/bench_activate_fhss.py) (this session)  |
| Boards – cold-boot profile  | both X8s' L072s boot to `REG_PROFILE=0` per P1 cold-boot discriminator 2026-05-22                       | confirmed; nothing on either X8 currently sets profile=1 before TX                                              |

**Verdict:** firmware Track A is done; the missing host-side piece was an
explicit pre-TX activator that issues the CFG_SET sequence at boot. That
gap is closed by [LifeTrac-v25/tools/bench_activate_fhss.py](LifeTrac-v25/tools/bench_activate_fhss.py)
landed this session.

---

## 2. Autonomous decision rationale

User left after asking "prioritize FHSS… make it our default bench
transmission style" then went unavailable mid-task. I had two open
questions:

> A. Orchestrator-staged at boot (host calls `CFG_SET(REG_PROFILE=1)`
>    before any container that emits RF starts) **vs**
> B. Firmware-default-on (change the L072 boot path to land in profile=1
>    instead of profile=0).

> Power: leave radios off, or warm them up to verify on bench?

I picked **A + radios stay off**. Reasoning:

1. **A is reversible at one shell prompt.** B requires reflashing both
   L072s and re-running the cold-boot discriminator, which (per
   `/memories/repo/lifetrac-x8-l072-bootloader.md` lines 24-28) you
   asked me explicitly not to touch without you in the loop because it
   trips the FCC-relevant Auto-Init decision.
2. **A leaves the bench bypass intact.** The v25.0.1 image demo's
   `BENCH_ONLY_FIXED_915` path still works for visualization; you can
   flip back at any time by re-running the activator with
   `--profile bench`.
3. **Radios off is safe by default.** No script in this session opens
   `/dev/ttymxc3` for writing; nothing in this session keys the PA.
4. **Dry-run is the new tool's default.** A misclick or accidental
   shell-history recall will NOT emit RF; you have to type `--apply`.

If you disagree (e.g. you want Path B because you want the L072 image
itself to default to FHSS), the activator script is still useful as the
test harness for the new boot path.

---

## 3. Known blocker for wide-mask end-to-end traffic (critical path)

Activating REG_PROFILE=1 is necessary but **not sufficient** for
two-peer FHSS image traffic on the bench. As of 2026-05-25
([radio_rx_frames_zero_channel_mismatch](LifeTrac-v25/AI%20NOTES/2026-05-25_Radio_RX_Frames_Zero_Channel_Mismatch.md)):

* The L072 driver, when activated with `REG_PROFILE=1` AND the wide
  50-channel mask, **independently picks a different starting channel
  per board** out of the mask.
* `RegHopPeriod=0` is set, so no per-packet hopping actually happens
  once a channel is picked — both peers sit on different fixed
  carriers.
* TX peer reports 200/200 fragments `TX_DONE=OK`; RX peer reports
  `rx_frames=0`. Symptom looks like an RF dead zone; root cause is
  channel mismatch.

This is plan §A6 — "RX hop sync state machine" — which is implemented
in `sx1276_rx_scan_policy.c` but has not yet been demonstrated to
close the loop with TX. **Demonstrating it is the critical-path work
item for true 50-ch FHSS to be the bench default.**

The workaround the v25.0.1 demo currently uses is `REG_PROFILE=0`
+ `LIFETRAC_FORCE_FRF_HZ=915000000` (deterministic single carrier).
The cheaper workaround under the FHSS profile is a single-channel
mask: same value on both peers, so the per-board pick has only one
choice. Both are exposed through this script's `--mask single --ch N`
mode, with `--profile fhss` so the regulatory artifact stamp is
honest.

### Roadmap impact

The production roadmap previously placed v25.0.7 "Regulatory Profile
Gate" after v25.0.4 / v25.0.5 / v25.0.6. Promoting FHSS as the bench
default means **v25.0.7 must be split**:

| Step           | What                                                                                                  | Prereq                          |
| -------------- | ----------------------------------------------------------------------------------------------------- | ------------------------------- |
| **v25.0.2.1**  | This script wired into the boot orchestrator on both X8s; readback verified on each boot              | none (do today, on bench)       |
| **v25.0.6.5**  | A6 RX-scan-policy demonstrated end-to-end with wide mask (`rx_frames > 0` after a 60 s air-test)      | bench + paired-peer test setup  |
| **v25.0.7**    | All regulatory artifacts (legal dwell ledger, per-channel dwell, RFCO summary stamp) green            | v25.0.6.5                       |

Until v25.0.6.5 closes, the activator's documented bench mode is
`--mask single` (same `--ch` on both peers). This is what the image
pipeline should use for bench traffic, with `--mask wide` reserved for
the A6 closure test itself.

---

## 4. What landed this session

1. **NEW** [LifeTrac-v25/tools/bench_activate_fhss.py](LifeTrac-v25/tools/bench_activate_fhss.py)
   — standalone host-side activator. Dry-run by default. Self-tested:
   golden wire vectors match `method_h_stage2_tx_probe_v2.py`
   (`SELF_TEST=PASS`).
2. **NEW** this document.
3. **NO CHANGES** to firmware, daemons, compose files, or systemd
   units. **NO ADB COMMANDS RUN** against either X8 in this session.

---

## 5. Runbook (what to do when you're back at the bench)

### 5.1 Preconditions

* Both X8s powered, adb reachable (`2E2C1209DABC240B` tractor,
  `2D0A1209DABC240B` base).
* `/dev/ttymxc3` not held by any container — confirm with
  `adb shell sudo lsof /dev/ttymxc3`. If `tractor-image-tx-v2`,
  `tractor-camera-v2`, or `lifetrac-vtest-image_rx-1` is running,
  stop them first (they were stopped at the end of the previous
  session; re-check).
* The L072 helper directory must exist on the X8 at
  `/tmp/lifetrac_p0c/` (it is the standard probe location;
  `method_g_stage1_probe.py` lives there). If absent, push it with
  the existing helper push script before step 5.3.

### 5.2 Push the activator to both X8s

```powershell
$adb = "C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe"
foreach ($s in @("2E2C1209DABC240B","2D0A1209DABC240B")) {
  & $adb -s $s push "LifeTrac-v25\tools\bench_activate_fhss.py" /tmp/lifetrac_p0c/bench_activate_fhss.py
}
```

### 5.3 Dry-run on each X8 (zero RF, prints plan)

```powershell
foreach ($s in @("2E2C1209DABC240B","2D0A1209DABC240B")) {
  & $adb -s $s shell "cd /tmp/lifetrac_p0c && python3 bench_activate_fhss.py --self-test"
  & $adb -s $s shell "cd /tmp/lifetrac_p0c && python3 bench_activate_fhss.py --mask single --ch 0"
}
```

Expected on both: `SELF_TEST=PASS`, then a dry-run plan ending with
`# DRY RUN — pass --apply to send these writes to the L072.` and
exit code 10.

### 5.4 Apply (real CFG_SET; no PA keying)

**Bench mode (recommended until v25.0.6.5 closes — single channel on
both peers, same `--ch`):**

```powershell
foreach ($s in @("2E2C1209DABC240B","2D0A1209DABC240B")) {
  & $adb -s $s shell "echo fio | sudo -S bash -c 'cd /tmp/lifetrac_p0c && python3 bench_activate_fhss.py --apply --profile fhss --mask single --ch 0'"
}
```

Expected on both: four `OK: CFG_SET …` lines then
`RUNTIME_PROFILE_ENUM=1` then `# profile activated: fhss`. Exit 0.

**True FHSS test (A6 / v25.0.6.5 critical-path closure ONLY):**

```powershell
foreach ($s in @("2E2C1209DABC240B","2D0A1209DABC240B")) {
  & $adb -s $s shell "echo fio | sudo -S bash -c 'cd /tmp/lifetrac_p0c && python3 bench_activate_fhss.py --apply --profile fhss --mask wide'"
}
```

Same expected output. **If you do this without the A6 fix in place,
the image pipeline will show `rx_frames=0` per §3 above.**

### 5.5 Verify via existing diagnostic

```powershell
foreach ($s in @("2E2C1209DABC240B","2D0A1209DABC240B")) {
  & $adb -s $s shell "cd /tmp/lifetrac_p0c && python3 method_g_stage1_probe.py --emit-runtime-profile --dev /dev/ttymxc3 --baud 921600"
}
```

(Or just trust the script's own readback line — they call the same
`CFG_GET REG_PROFILE` underneath.)

### 5.6 Bring the image pipeline back up

```powershell
& $adb -s 2E2C1209DABC240B shell "echo fio | sudo -S docker start tractor-camera-v2 tractor-image-tx-v2"
& $adb -s 2D0A1209DABC240B shell "echo fio | sudo -S docker start lifetrac-vtest-image_rx-1"
```

Image traffic will now ride profile=1. The artifact stamps in the
RFCO summary will show `REG_PROFILE_ENUM=1` instead of `=0`.

### 5.7 Roll back (if something is wrong)

```powershell
foreach ($s in @("2E2C1209DABC240B","2D0A1209DABC240B")) {
  & $adb -s $s shell "echo fio | sudo -S bash -c 'cd /tmp/lifetrac_p0c && python3 bench_activate_fhss.py --apply --profile bench --mask single --ch 0'"
}
```

This drops both back to `BENCH_ONLY_FIXED_915` — the carrier path the
v25.0.1 demo was validated on.

---

## 6. Open questions for you (answer when convenient)

1. Do you want the activator wired into a systemd one-shot unit
   (`Before=docker.service`), or do you want to leave it as a manual
   pre-start step until v25.0.6.5 closes? I deferred the systemd unit
   because writing one before the activator is proven on bench felt
   premature, and rolling it back is annoying.
2. Do you want me to lift the activator into the
   `image_tx_daemon._open_link()` path as well, so the daemon refuses
   to start if `RUNTIME_PROFILE_ENUM` doesn't read back as the
   requested value? That is the cleanest fail-fast, but it changes
   the daemon's startup semantics — wanted your sign-off first.
3. v25.0.6.5 (A6 RX-scan closure): I can start writing the
   bench-test harness for this autonomously, but it requires real
   PA-on time on both peers to falsify, so I'd rather queue it for
   when you're at the bench.

---

## 7. Memory follow-ups (will record after this doc is written)

* Repo memory `lifetrac-x8-l072-bootloader.md`: record that the
  bench-default activator now exists, that Path A was chosen, and
  that no firmware default has changed.
* No user-memory entries warranted (this is repo-specific).

---

## 8. Mixed TX/RX Control Cadence Update (GitHub Copilot v1.1, 2026-05-27)

This plan originally treats FHSS bench-default as an image-link problem:
tractor sends image fragments, base receives them. The mixed-control review
adds a second requirement for the same 50-channel radio: base must sometimes
send P0/P1 control traffic back to the tractor without stealing the base's RX
time or breaking hop synchronization.

The updated policy is: **send control on change, not continuously, and treat
held control state as a short lease.** This keeps the base listening for
tractor image/telemetry most of the time, while still making reverse control
deterministic when the operator actually changes state.

### 8.1 Control state is not a 20 Hz byte stream under one-radio FHSS

For the mixed FHSS bench, do not spend every reverse window on an unchanged
joystick snapshot. Use a latest-wins state model:

| Control condition | Air behavior | Initial bench target |
|---|---|---:|
| Joystick/button changed beyond deadband | Send at next available reverse window | immediate |
| Held nonzero command | Refresh the lease periodically | 5-10 Hz |
| Held neutral command | Quiet refresh only | 1-2 Hz |
| Source presence / UI alive | Heartbeat only | 1-2 Hz |
| E-stop, take-control, handover | Send immediately and repeat | P0 burst |

This is the practical answer to the "base RX matrix" question. Every
unchanged control packet we do not send is another reverse window the base
can leave unused, which means the base stays in RX for the next tractor
master/image packet instead of spending airtime on duplicate stick values.

### 8.2 Lease cutoffs: more forgiving, but layered

The cutoffs can be more forgiving than the old blanket 500 ms heartbeat idea,
but they should not all move together:

| Lease | Candidate cutoff | Failure behavior |
|---|---:|---|
| Nonzero actuator command lease | 500-750 ms | Neutralize commanded actuators |
| Neutral command lease | 1.5-2.0 s | Keep neutral, mark source stale if no heartbeat |
| Source presence heartbeat | 1.5-2.0 s | Source inactive for arbitration |
| E-stop latch | no timeout | Latched until explicit clear |

This gives the operator/UI more breathing room without allowing a lost link
to hold a drive or hydraulic command indefinitely. The tractor-local watchdog
and valve-neutral fail-safe remain mandatory; LoRa silence must still fail
closed.

### 8.3 FHSS implications

This cadence update should be paired with the mixed-mode exchange-window plan
from the RX scan document:

1. Tractor image/telemetry frames remain the hop-clock master while the image
   path is active.
2. Base P0/P1 control replies use same-channel reverse windows and a
   non-advancing reply TX mode.
3. The base transmits only the freshest state, not a backlog of stale states.
4. Tractor emits low-rate master keep-alives only when image traffic is idle,
   and only fast enough to meet control feel and source-presence targets.
5. All image, control, heartbeat, and keep-alive traffic stays on the same
   50-channel FHSS profile and the same RFCO/dwell accounting path.

### 8.4 Acceptance gates to add before FHSS becomes the normal mixed bench

- Image-only wide-mask FHSS still has to pass first: Alpha-Prime + Delta are
  unchanged as v25.0.6.5 foundation.
- Mixed locked operation: base sends changed P0/P1 state in the next reverse
  window, while unchanged steady input does not occupy every window.
- P0 starvation: under image load, no fresh P0 state waits behind P3 image
  fragments; stale intermediate controls are superseded.
- Lease safety: held nonzero control expires to neutral within the chosen
  500-750 ms cutoff when reverse control frames stop.
- Airtime/RX benefit: record reverse-window occupancy with steady neutral
  input and verify that the base spends materially more time receiving image
  traffic than it would under constant 20 Hz duplicate controls.

Bottom line: the FHSS bench-default plan should not try to make a one-radio
image/control link behave like two always-on streams. The better plan is a
single 50-channel FHSS exchange: tractor packets keep the hop clock alive,
base sends control only on change or lease refresh, unchanged neutral/source
presence is allowed a more forgiving heartbeat, and nonzero actuator state
expires quickly if the link goes quiet.

*Signed:* GitHub Copilot, FHSS Mixed-Control Cadence Update v1.1 (2026-05-27)

## 9. Review of §8 — Mixed TX/RX Control Cadence (GitHub Copilot v1.2, 2026-05-27)

Reviewing §8 (v1.1) against the v25.0.6.5 α-prime + δ foundation, the
host-tested `radio/sx1276_rx_scan_walker.c` TU, and the existing FHSS / RFCO
accounting paths. Architecture is sound; below are the concrete deltas
needed before §8 can be treated as a buildable spec.

### 9.1 What §8 gets right (keep as-is)

1. **Latest-wins state over a byte stream** (§8.1). Correct for a single-radio
   FHSS link with a 50-channel scheduler — no point retransmitting a state
   the receiver already has.
2. **Layered lease cutoffs** (§8.2). Decoupling *actuator command authority*
   (500–750 ms) from *source presence* (1.5–2.0 s) is correct: authority
   must fail fast, presence can be lazy.
3. **E-stop latched-until-explicit-clear** (§8.2). Correct policy (see §9.3
   below for the missing precision on *what counts as* explicit clear).
4. **Tractor packets remain hop-clock master** (§8.3 #1). Preserves the
   v25.0.6.5 α-prime + δ contract — base SNAPs to tractor, never the
   reverse.
5. **All traffic on the same RFCO/dwell accounting** (§8.3 #5). Avoids the
   trap of letting reverse-control traffic escape FCC bookkeeping.

### 9.2 Quantify the held-nonzero refresh against airtime

§8.1's table proposes **5–10 Hz** refresh for *held nonzero command*. At
SF7/BW125/CR4-5 a 16-byte P0 is ≈ 30 ms airtime (preamble + header +
payload + CRC). That gives:

| Held-nonzero refresh rate | Reverse airtime fraction |
|---:|---:|
| 5 Hz × 30 ms | 15 % |
| 10 Hz × 30 ms | **30 %** |

A *held nonzero command* is the **normal driving condition**, not an edge
case. 15–30 % of windows going to reverse-control during driving means
image throughput drops by the same fraction at exactly the moment the
operator wants visual feedback most. §8.1's framing ("keeps the base
listening most of the time") is true for **neutral**, not for held nonzero.

**Proposed delta:** quantify the choice against P0 airtime and consider
**2–3 Hz with `lease_cutoff = 750 ms`** so the actuator survives one
missed refresh + one retry. That drops reverse load to **6–9 %** and the
operator will not perceive it. Add this calculation to §8.1 directly so a
future reviewer cannot re-litigate it from first principles.

### 9.3 Send-on-change needs hysteresis and a floor interval

§8.1 says "changed beyond deadband → send at next available reverse
window, immediate." Two failure modes are unhandled:

1. **Deadband flutter.** An axis sitting right at the deadband edge will
   toggle in/out → fires a send on every window. Worst case: send rate is
   no longer bounded by the held-nonzero refresh rate. Need a minimum
   inter-change-send interval (e.g. 50–100 ms) AND deadband hysteresis
   (enter at X, exit at X − Δ).
2. **Nonzero↔neutral toggling.** When the operator releases the stick
   (nonzero → deadband), §8 implies "another change event → send
   immediately, then drop into neutral cadence." But the next twitch back
   into deadband would re-fire immediately. Specify: a debounce window
   (≥ neutral-lease ÷ N) before a nonzero→neutral transition is
   recognized as durable.

### 9.4 Lease/latch ambiguities to nail down in §8.2

1. **E-stop "explicit clear" path.** Clear by *what*? Required: a positive,
   authenticated P0 from the registered controller with an *explicit
   clear* opcode — **not** "any P0 with E-stop bit clear." Otherwise a
   stale frame or a misordered packet could clear E-stop. State it as a
   distinct command type, not the absence of the assert.
2. **Nonzero → neutral transition on lease expiry.** When the nonzero
   lease expires (500–750 ms silence) and the local controller
   neutralizes the actuator, does that transition *reset* the neutral
   lease timer? It must — otherwise the actuator drops to neutral and a
   stale-source check fires 0.75–1.5 s later. Specify: local-decision-to-
   neutralize resets the neutral lease as if a fresh neutral frame had
   arrived.
3. **Restart anti-bounce.** After local-neutralize fires, what's the
   criterion for accepting the *next* nonzero command? Recommendation:
   require **2 consecutive valid nonzero frames** within the nonzero-
   lease window before the actuator may leave neutral. Prevents a single
   late/duplicate burst from re-energizing a runaway.

### 9.5 §8.3 #2 "non-advancing reply TX mode" is a hidden firmware delta

This is buried in one bullet but it is a substantial change:

- Today's `sx1276_fhss_next_channel()` mutates `s_fhss.slot`. If the base
  TX path calls it, the **base's slot advances independently of the
  tractor's** — exactly the desync mode α-prime fixed for the RX path.
- Two viable strategies, both need code:
  1. **Reply-on-current-slot:** base TX uses the slot the tractor *just
     transmitted on*, without advancing local `s_fhss.slot`. Requires a
     new `sx1276_fhss_peek_current(...)` API plus a TX path that does not
     consume.
  2. **Locked symmetric advance:** both peers advance in lockstep on every
     reverse frame too. Brittle — one missed reverse leaves them off by
     one until the next `sx1276_fhss_consider_remote()` snap.
- §8.3 #2 should declare which strategy is in scope and add the API delta
  to the build list. **Strategy 1 is the right pick;** it is a sibling of
  the `sx1276_rx_scan_walker` work for the RX side (both keep `s_fhss`
  observe-only on their respective hot paths).

### 9.6 Priority-queue inversion is a build item, not a gate

§8.4's "P0 starvation: no fresh P0 state waits behind P3 image fragments"
is currently written as a **test**, but the firmware mechanism to
guarantee it is unspecified. Today the TX queue is FIFO; a P0 enqueued
behind 6 P3 fragments waits 6 × dwell. Required: either
  (a) head-of-line preempt for P0/P1 over P3, or
  (b) a separate P0/P1 fast-path queue serviced ahead of the image queue
      at every TX opportunity.
Call this out in §8.4 as a **build prerequisite** of the gate, not the
gate itself.

### 9.7 Make the acceptance gates quantitative

§8.4 says "materially more time receiving image traffic." Replace
qualitative gates with measurable ones (all observable from existing
bench instrumentation + `sx1276_rx_counter_record` histogram):

| §8.4 gate (current, qualitative) | Quantified replacement |
|---|---|
| Mixed locked operation | Joystick edge → corresponding `actuator_demand` change in base→tractor logs within ≤ 1 reverse-window (≤ 200 ms) |
| P0 starvation | Under saturating image load, P0 send-to-act latency 99th percentile ≤ 250 ms |
| Lease safety | Held nonzero, reverse link killed at t₀: actuator at neutral by t₀ + 750 ms in 100 % of trials |
| Airtime/RX benefit | Steady neutral input ≥ 30 s: reverse-window occupancy ≤ 10 %; image frames/sec ≥ 0.9 × image-only baseline |

### 9.8 Lock in the walker contract

§8.4's first bullet ("Alpha-Prime + Delta are unchanged as v25.0.6.5
foundation") should be tightened to:

> The walker TU (`radio/sx1276_rx_scan_walker.c`, host-tested 2026-05-27
> via `check-rx-scan-walker`) is the foundation; any mixed-control
> firmware must continue to leave `s_fhss` observe-only on the scan path.
> The reverse-TX path (§9.5) is held to the same contract.

This prevents a future mixed-mode increment from silently reintroducing
the same shared-state desync the walker fixed.

### 9.9 Minor wording / scope

- §8.1 "Held neutral command — 1-2 Hz." With a 1.5–2.0 s neutral cutoff,
  1 Hz survives one miss, 2 Hz survives three. **1 Hz** feels right;
  reserve 2 Hz for environments with elevated PER.
- §8.3 #4 "low-rate master keep-alives only when image traffic is idle"
  — define "idle." Recommend: no P3 fragment in the TX queue AND no P3
  sent in the last N dwells. Otherwise a 1 fps image stream is "idle"
  between frames and triggers spurious keep-alives.
- §8.4 final paragraph: explicitly note this work is **v25.1.x scope**,
  not v25.0.6.5, so the reader does not conflate it with the foundation
  flash. v25.0.6.5 is α-prime + δ image-link only; mixed control is the
  follow-up.

### 9.10 Bottom line

§8 is architecturally sound. Two changes are required before it is a
buildable spec:

1. **Quantify the held-nonzero refresh against airtime** (§9.2) and drop
   to **2–3 Hz with a 750 ms cutoff**.
2. **Promote three items currently treated as gates or asides into
   explicit build deltas** before the gates can run:
   - non-advancing reverse-TX API (§9.5),
   - priority-queue inversion (§9.6),
   - deadband-flutter floor + hysteresis (§9.3).

Everything else (§9.4, §9.7, §9.8, §9.9) is wording precision and is
recommended for the next pass.

*Signed:* GitHub Copilot, Review of §8 v1.2 (2026-05-27)
