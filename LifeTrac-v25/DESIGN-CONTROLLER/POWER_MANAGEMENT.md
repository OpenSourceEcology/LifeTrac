# Power Management — production power-down / wake design

**Status: design draft, 2026-08-22. No implementation yet.** Origin: bench discussion
during the RS-12.9 radio session. Companion to [TRACTOR_NODE.md](TRACTOR_NODE.md)
(hardware + Modbus map) and [BASE_STATION.md](BASE_STATION.md).

## Requirements

- In production the tractor may run for a day, then sit **completely powered off for
  weeks** (master cutoff open). No operator shutdown ceremony beyond turning the key.
- The Linux side (Portenta X8) must always halt cleanly — eMMC/OSTree must never see
  an unplanned power cut in the normal key-off path.
- The base station stays on mains power and should **quiesce** while the tractor is
  away, then resume automatically when it returns.
- **Bench radio testing must be unaffected**: no ignition wiring, no Opta, and no
  power-signal timeouts on the bench. See "Enable gating" below.

## Tractor: sensing key-off

The sensing hardware and register plumbing already exist in the design — this doc
adds the consumer, not the inputs:

| Signal | Source | Modbus register |
|---|---|---|
| Ignition present (digital) | Opta expansion input **I1 — ignition sense** ([TRACTOR_NODE.md](TRACTOR_NODE.md), I/O map) | `digital_inputs` 0x0101 |
| Battery rail voltage (analog) | **AI1 — battery voltage**, native 0–30 V range on the A0602 expansion (no external divider) | `battery_mv` 0x0102 |

Power topology (already consistent with the TRACTOR_NODE.md wiring tree): the Opta
and Max Carrier are fed from the constant (master-cutoff-side) distribution, **not**
key-switched power. The ignition circuit lands only on I1. The Opta therefore stays
alive through the grace window and can report key-off; "weeks off" is the master
cutoff opening after shutdown completes.

### Who owns the decision

The **Max Carrier H747 co-MCU** owns the debounce and the shutdown decision. It is
the always-alive component (it survives Linux crashes and reboots by design) and it
already polls the Opta as Modbus master, so it sees I1 and `battery_mv` every cycle.

Debounce rule (initial proposal, tune on the bench):

- `ignition lost` = I1 low for **10 s continuous**. Cranking sags and stalls do not
  clear the timer via voltage — I1 is a key-position signal, not a rail level.
- `battery_mv` disambiguates: key-off shows I1 low with a healthy rail; a genuine
  electrical failure shows the rail collapsing with I1 still high. The second case
  should trigger an *urgent* shutdown (shorter debounce) plus a telemetry alarm.
- Modbus silence from the Opta keeps its existing meaning (safety event → coils
  drop via the Opta's own watchdog). It is **never** interpreted as key-off.

### Linux shutdown sequence (on H747 command)

1. Hydraulic outputs are already safe (Opta watchdog + PSR chain — not this doc's
   job, and never gated by it).
2. Stop image/radio containers; park the LoRa radio in sleep.
3. Send a **farewell packet** to the base (see "Base station" below).
4. `sync` + clean halt.
5. Power release (see below).

The 18650 in the Max Carrier's UPS slot only has to bridge steps 2–4 (≈30–60 s),
not the weeks-off period.

### Power release after halt

Preferred: a **self-holding power relay** — energized at key-on, held by an X8/H747
output, released when halt completes. This makes power-off fully automatic, removes
any dependence on the operator throwing the master cutoff promptly, and shrinks the
18650's duty to the bridge window only. Fallback: operator opens the master cutoff;
acceptable, but then post-halt battery drain matters (below).

### Battery caveats (must be resolved before production)

- **Post-halt drain measurement.** After a halt with the master cutoff still closed,
  measure the current out of the 18650. At ~1 mA a 3000 mAh cell survives months; at
  ~20 mA it deep-discharges (and is damaged) in about a week. If high, use the
  charger's ship/hibernate mode or a load switch, or adopt the self-holding relay.
- **Cold charging.** The tractor lives outdoors; charging Li-ion below 0 °C damages
  the cell and the charger will attempt it at first key-on in winter. Require a
  temperature-gated charge path (NTC/JEITA), or substitute **LiFePO4 or a
  supercapacitor module** — for a 60 s bridge, a supercap has no chemistry to
  maintain and no cycle wear, and is probably the better production part.
- **Crash-only hardening as belt-and-braces:** read-only rootfs posture (the X8's
  OSTree base is close already) + journaled data partition, so a failed graceful
  path or yanked cutoff is still survivable.

### Open bench checks

- [ ] Does the X8 kernel expose the Max Carrier charger's "adapter present" status
  under `/sys/class/power_supply/`? (Free to check while boards are idle; would
  serve as a backstop signal to the Opta path.)
- [ ] Post-halt 18650 drain measurement (needs a meter in series, some evening).
- [ ] Confirm the ignition-sense circuit lands on I1 in the harness drawing as the
  I/O map assumes.

## Enable gating — bench mode is the default

The whole power-management consumer is **default OFF** and enabled explicitly in
production provisioning:

- Setting (name provisional): `LIFETRAC_POWER_MGMT=1` — reaches the H747 via its
  config path at provisioning time; documented in
  [SETTINGS_REFERENCE.md](SETTINGS_REFERENCE.md) once implemented.
- **Absence of the flag *is* bench mode.** With the flag unset: no ignition/VIN
  logic is armed, no shutdown timers run, and the radios can be exercised
  indefinitely with no Opta attached and no power signals present. No separate
  "bench mode" variable is needed, and the bench default is unchanged by shipping
  the feature.
- This matches the established pattern for behavior changes on this bench
  (`LIFETRAC_NO_PARK_LAST` is env-gated, not default).
- Failure direction is deliberate: a misprovisioned flag fails toward *staying up*
  (annoying, visible, recoverable) rather than toward surprise shutdowns.
- Production additionally gets a **maintenance override** (config or physical
  jumper) to keep the node up with the key off for field diagnostics.

**Scope guard:** this flag governs *orderly Linux shutdown only*. The Opta coil
watchdog, the H747 radio-liveness watchdog, and the PSR safety chain are always
armed regardless of this setting. Power management must never be in the valve-safety
path.

## Base station while the tractor is away

- On receiving the tractor's **farewell packet**, the base stops the image pipeline
  and drops to low-duty listen. The web UI stays up and shows "tractor off
  (expected)" — distinct from lost-link, so nobody chases a phantom outage and the
  base does not burn airtime on retries.
- **Idle-time channel surveys.** RS-11.6 established that channel picks perish in
  hours and a same-day survey is a correctness requirement. A quiesced base runs
  the survey automatically every few hours (the RS-11.7 automation, extended), so
  the current clean-channel pick is always ready when the tractor returns.
- **Rendezvous protocol.** The tractor wakes with a weeks-stale channel pin. It
  hails on a fixed **home channel, 927.5 MHz** (the only 3/3-clean channel across
  the 2026-08 surveys; plausibly band-edge-protected). The base answers with
  today's channel assignment; both hop; the session resumes. Session start is
  therefore: hello on home channel → channel assignment → traffic.

## Out of scope (noted for later)

- Fast hydraulic availability at key-on: control on the H747 boots in milliseconds
  while Linux takes tens of seconds; this design only protects the Linux side and
  does not put boot time in the operator's way.
- Scheduled self-wake while parked (weekly health ping): rejected for now — it
  spends the bridge battery on a non-essential feature.
