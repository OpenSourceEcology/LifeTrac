# Power Management — production power-down / wake design

**Status: design draft, 2026-08-22, revised same day after PR #110 review.** No
implementation yet. Origin: bench discussion during the RS-12.9 radio session.
Companion to [TRACTOR_NODE.md](TRACTOR_NODE.md) (hardware + Modbus map) and
[BASE_STATION.md](BASE_STATION.md).

## Requirements

- In production the tractor may run for a day, then sit **completely powered off for
  weeks**. No operator shutdown ceremony beyond turning the key.
- The Linux side (Portenta X8) must always halt cleanly — eMMC/OSTree must never see
  an unplanned power cut in the normal key-off path.
- The base station stays on mains power and should **quiesce** while the tractor is
  away, then resume automatically when it returns.
- **Bench radio testing must be unaffected**: no ignition wiring, no Opta, and no
  power-signal timeouts on the bench. See "Enable gating" below.

## Tractor: sensing key-off

### Proposed signal mapping

The input hardware and register addresses exist in the design; the *producers and
consumers do not yet exist in firmware*. This is a *proposed* mapping:

| Signal | Source | Modbus register |
|---|---|---|
| Ignition present (digital) | Opta expansion input **I1 — ignition sense** ([TRACTOR_NODE.md](TRACTOR_NODE.md) I/O map) | a defined bit in `digital_inputs` 0x0101 |
| Battery rail voltage (analog) | **AI1 — battery voltage**, native 0–30 V range on the A0602 expansion (no external divider) | `battery_mv` 0x0102, after calibration |

**Current firmware state (verified against source, 2026-08-22):**

- The Opta writes `digital_inputs` from the E-stop loop (bit 0) and mode switch
  (bit 1) only (`tractor_opta.ino` input-refresh block); **I1 is not published**.
- The H7's telemetry poll requests only the six analog registers starting at
  0x0102 (`tractor_h7.ino` `emit_telemetry()`); **0x0101 is never read**.
- The analog block is forwarded as **raw ADC codes** — the mV scaling implied by
  the register name is still pending (tracked in TODO.md).

**Implementation prerequisites** (all must land before the debounce below is real):

- [ ] Opta publishes I1 into `digital_inputs` at a documented bit position (and the
  bitfield layout gets a table in [TRACTOR_NODE.md](TRACTOR_NODE.md)).
- [ ] H7 read block widened to cover 0x0101 (start the request at 0x0100/0x0101 or
  add a second request).
- [ ] `battery_mv` calibrated to actual millivolts with a recorded scale factor,
  and the key-off/failure thresholds defined against it.

### Power topology

Consistent with the TRACTOR_NODE.md wiring tree: the Opta and Max Carrier are fed
from the constant (battery-side) distribution, **not** key-switched power. The
ignition circuit lands only on I1. The Opta therefore stays alive through the grace
window and can report key-off.

### Who owns the decision

The **Max Carrier H747 co-MCU** owns the debounce and the shutdown decision. It is
the always-alive component (it survives Linux crashes and reboots by design) and it
already polls the Opta as Modbus master — once the read block is widened per the
prerequisites above, it sees I1 and `battery_mv` every cycle.

Debounce rule (initial proposal, tune on the bench):

- `ignition lost` = I1 low for **10 s continuous**. Cranking sags and stalls do not
  clear the timer via voltage — I1 is a key-position signal, not a rail level.
- `battery_mv` disambiguates: key-off shows I1 low with a healthy rail; a genuine
  electrical failure shows the rail collapsing with I1 still high. The second case
  triggers an *urgent* shutdown (shorter debounce) plus a telemetry alarm.
- **Failure-mode caveat (from review):** `battery_mv` arrives over the same Modbus
  link and Opta supply it is meant to diagnose — a collapse that kills the Opta
  yields Modbus silence, not a low sample. Therefore: (a) the H747 keeps
  **last-known-value semantics** with an explicit staleness bound (proposal: a
  sample older than 1 s is not evidence of anything), and (b) the urgent-shutdown
  path needs an **H747/carrier-side rail measurement independent of the Opta** —
  candidates are the Max Carrier charger's "adapter present" status or a carrier
  ADC on VIN; confirming what is actually readable is a bench check below.
- Modbus silence from the Opta keeps its existing meaning (safety event → coils
  drop via the Opta's own watchdog). It is **never** interpreted as key-off.

### H747 → Linux shutdown path (new protocol surface)

No power-management opcode exists on the X8↔H747 UART protocol today (the current
command path forwards camera-related commands only). Required additions:

- `SHUTDOWN_REQ` (H747 → Linux): carries a reason code (key-off / urgent-electrical
  / maintenance).
- `SHUTDOWN_ACK` (Linux → H747): "halting now"; absence handled by the release
  timeout below.

### Linux shutdown sequence (on `SHUTDOWN_REQ`)

1. Hydraulic outputs are already safe (Opta watchdog + PSR chain — not this doc's
   job, and never gated by it).
2. Send the **farewell packet** to the base *while the radio is still up*, with a
   bounded ack wait (proposal: up to 3 attempts / 10 s total). Proceed regardless
   of ack — power-down must never block on the base.
3. Stop image/radio containers; park the LoRa radio in sleep.
4. `sync` + clean halt.

(Review fix: farewell moved ahead of radio teardown — the original draft parked the
radio before transmitting, which cannot work.)

The 18650 in the Max Carrier's UPS slot only has to bridge this sequence (≈30–60 s),
not the weeks-off period.

### Power release after halt — handshake, not hope

Preferred topology: a **self-holding power relay** — coil energized at key-on
through the ignition circuit, held by an H747 output once booted, released after
halt. Release logic (from review — completion must be observable, and the timeout
must be safe):

1. H747 sends `SHUTDOWN_REQ`, expects `SHUTDOWN_ACK` within 15 s.
2. After ack, H747 watches the existing Linux liveness signal it already consumes;
   release fires **20 s after the last sign of Linux life** (halt has completed and
   settled).
3. **Fail-safe cap:** if no ack arrives, or liveness never ceases, release fires at
   **300 s** after `SHUTDOWN_REQ` regardless. This path can cut power mid-write,
   which is exactly why **crash-only rootfs hardening (read-only rootfs posture +
   journaled data partition) is a prerequisite for adopting the self-holding
   topology**, not an optional extra.

Fallback topology: operator opens the master cutoff; acceptable, but then post-halt
battery drain matters (below).

### Wake path (from review — the key must actually be able to wake the node)

- **Normal wake:** key-on energizes the hold-relay coil *directly through the
  ignition circuit* (diode-OR'd with the H747 hold output). The node boots on key
  power, the H747 latches the hold, and the ignition diode leg becomes redundant
  until the next cycle. Weeks-parked standby draw is then the relay-coil leakage
  path only (spec ~0), not any electronics.
- **Master cutoff open** (service/long storage): everything is dead including the
  wake path; **closing the cutoff is a required, documented step** before the key
  works. The cutoff is a service disconnect, not the routine off switch.

### Battery caveats (must be resolved before production)

- **Post-halt drain measurement.** After a halt with power still connected, measure
  the current out of the 18650. At ~1 mA a 3000 mAh cell survives months; at ~20 mA
  it deep-discharges (and is damaged) in about a week. If high, use the charger's
  ship/hibernate mode or a load switch — or the self-holding relay makes this moot.
- **Cold charging.** The tractor lives outdoors; charging Li-ion below 0 °C damages
  the cell and the charger will attempt it at first key-on in winter. Require a
  temperature-gated charge path (NTC/JEITA), or substitute **LiFePO4 or a
  supercapacitor module** — for a 60 s bridge, a supercap has no chemistry to
  maintain and no cycle wear, and is probably the better production part.
- **Crash-only hardening:** read-only rootfs posture (the X8's OSTree base is close
  already) + journaled data partition. Required for the self-holding topology (see
  above); strongly recommended regardless.

### Open bench checks

- [ ] Does the X8 kernel expose the Max Carrier charger's "adapter present" status
  under `/sys/class/power_supply/`? (Free to check while boards are idle; feeds the
  independent-rail-measurement requirement above.)
- [ ] Is there a carrier-side ADC path to VIN readable by the H747 or X8?
- [ ] Post-halt 18650 drain measurement (meter in series, some evening).
- [ ] Confirm the ignition-sense circuit lands on I1 in the harness drawing as the
  I/O map assumes.

## Enable gating — bench mode is the default

The whole power-management consumer is **default OFF** and enabled explicitly in
production provisioning. From review: an X8 environment variable cannot arm the
bare-metal H747, so the gate is a *persisted parameter*, not an env var:

- The flag lives in the **parameter service** (persisted JSON), and is delivered to
  the H747 at boot over the existing parameter path. Name provisional:
  `power_mgmt_enabled`; documented in [SETTINGS_REFERENCE.md](SETTINGS_REFERENCE.md)
  once implemented.
- **H747 default is DISARMED.** If the parameter is never delivered (bench images,
  fresh boards, param service down), no ignition/VIN logic runs. Absence of the
  flag *is* bench mode — no separate bench variable, and the radios can be
  exercised indefinitely with no Opta attached and no power signals present.
- A Linux-side `LIFETRAC_POWER_MGMT` env var may mirror the parameter for the
  container-side pieces (farewell sender, halt hook), but the H747 arms only from
  the persisted parameter.
- Failure direction is deliberate: a misprovisioned flag fails toward *staying up*
  (annoying, visible, recoverable) rather than toward surprise shutdowns.
- Production additionally gets a **maintenance override** (config or physical
  jumper) to keep the node up with the key off for field diagnostics.

**Scope guard:** this flag governs *orderly Linux shutdown only*. The Opta coil
watchdog, the H747 radio-liveness watchdog, and the PSR safety chain are always
armed regardless of this setting. Power management must never be in the valve-safety
path.

## Base station while the tractor is away

### Farewell, ack, and the lost-farewell case

The farewell is a **new protocol frame** (to be specified in
[LORA_PROTOCOL.md](LORA_PROTOCOL.md); authenticated once the AEAD work lands), with
a base-side ack:

- **Acked farewell** → base enters **expected-off**: image pipeline stops, low-duty
  listen, web UI shows "tractor off (expected)".
- **Lost farewell** (tractor halts after its bounded retries without an ack) → the
  base keeps its existing lost-link behavior (banner, control traffic stops). The
  operator-visible distinction between "expected off" and "lost link" is preserved
  precisely because only an acked farewell produces the expected-off state; silence
  never does. A base in lost-link state for a long period behaves identically to a
  quiesced one for radio purposes (it is listening either way) and resolves to
  expected-off retroactively if the tractor's next hello says "I shut down cleanly
  at T".

### Idle-time channel surveys

RS-11.6 established that channel picks perish in hours and a same-day survey is a
correctness requirement. A quiesced base runs the survey automatically every few
hours, so the current clean-channel pick is ready when the tractor returns.

Status honesty (from review): RS-11.7 as currently proposed is a *manual* v0
(operator-triggered, APPLY is manual, RX daemon paused during the sweep). The
periodic automation described here is an extension that must define **hail handling
during survey dwells**: the quiesced base alternates survey passes with hail-listen
windows, and a full survey pass is bounded (≈30 s × channels) while the tractor's
hail loop retries for much longer (minutes, below) — so a wake hail that lands
mid-survey is caught on the next listen window, at the cost of bounded extra wake
latency, never stranding.

### Rendezvous — hail set, not a single home channel

The tractor wakes with a weeks-stale channel pin, so session start is:
**hello on the hail set → channel assignment from the base → hop → traffic.**

From review, a single fixed rendezvous channel is fragile and the specific bench
pick doesn't transfer as-is:

- **Hail SET, not one channel:** 2–3 channels chosen from survey-history *stability*
  ranking (the `survey_compare.py --history` clean-in-N-of-M logic), refreshed
  whenever the fleet is re-provisioned. The tractor cycles hails across the set
  with backoff (proposal: full-set pass every ~10 s, for ≥5 min before declaring
  no-base); the quiesced base's listen windows scan the same set. Two peers that
  share the set cannot be stranded by one channel going hot.
- **Grid alignment caveat:** the production FHSS channel table is 50 channels with
  centers 902.75–927.25 MHz (see SETTINGS_REFERENCE.md, `sx1276_fhss_chantab.h`).
  The bench surveys that found 927.5 MHz clean ran on the offset x.0/x.5 grid —
  **927.5 is not a table channel, and no table channel has bench stability data
  yet.** The band-edge-stability *finding* motivates picking hail channels near the
  edges; the actual constants must come from surveys run on the chantab grid (or a
  deliberate table extension, which is a regulatory-review item, not a default).
- **Mode transitions:** the tractor enters hail mode at every boot-with-no-session
  and after sustained link loss (reusing the existing link-loss criterion); it
  exits on an acked channel assignment. The base services hail-listen whenever it
  has no active session (quiesced *or* lost-link — same behavior, see above).

## Out of scope (noted for later)

- Fast hydraulic availability at key-on: control on the H747 boots in milliseconds
  while Linux takes tens of seconds; this design only protects the Linux side and
  does not put boot time in the operator's way.
- Scheduled self-wake while parked (weekly health ping): rejected for now — it
  spends the bridge battery on a non-essential feature.
