# Automation & Route Planning — Future Release Scope

**Status:** Research / future-release scope. **Not** part of the v25 hardware-test build.
**Date:** 2026-04-26
**Position in repo:** filed under [`RESEARCH-CONTROLLER/`](.) per [MASTER_PLAN.md §6](../MASTER_PLAN.md#6-what-goes-where-going-forward) — autonomy is excluded from v25 readiness gates until promoted by an explicit MASTER_PLAN amendment.

This document parks autonomy and route-planning ideas in one place so they (a) stop creeping into v25 readiness reviews and (b) inform the *interfaces* the v25 controller must not foreclose.

Related archived material in this folder:

- [`PATH_PLANNING/`](PATH_PLANNING/) — earlier path-planning research notes
- [`ros2_bridge/`](ros2_bridge/) — ROS 2 ↔ MQTT bridge prototype (uses the legacy MQTT-over-WiFi path; salvageable as a base-station-side ROS 2 bridge)
- [`raspberry_pi_web_controller/`](raspberry_pi_web_controller/) — legacy web UI; some autonomy UI patterns reusable
- [`WIRELESS_OPTIONS.md`](WIRELESS_OPTIONS.md) — historical wireless comparison; relevant for "do we need a higher-bandwidth uplink for autonomy?" questions

---

## 1. Goals (future release)

Autonomy on LifeTrac is for **field operations the operator does not want to drive**: mowing, tilling, seeding rows, repeated transport between two points, headland turns. It is **not** for tasks that require fine human judgement (loader work, hitching, anything in tight quarters with people).

Concrete capabilities, in priority order:

1. **Path replay** — record a manually-driven path with the operator at the wheel; replay it later. Lowest autonomy bar; useful for repeated chore loops.
2. **Waypoint follow** — operator places GPS waypoints on the base-station map; tractor drives the polyline between them at a commanded speed.
3. **Coverage planning** — for a polygon (field boundary), generate a back-and-forth or boustrophedon coverage path with configurable swath width and headland turn radius.
4. **Headland / U-turn maneuvers** — proper turn geometry at field edges, optionally with implement raise/lower.
5. **Geofence + keep-out zones** — autonomy refuses to drive outside a polygon or into a keep-out polygon. Enforced on the tractor M4, not in the planner.
6. **Implement automation** — boom up/down, bucket position, PTO on/off at scripted points along the path.
7. **Multi-pass field operations** — load an ISOXML / GeoJSON / KML plan from farm-management software, execute and log.
8. **Obstacle stop** — camera or LiDAR-based obstacle detection that sends `CMD_ESTOP` (not steering); the tractor stops, operator decides.

Out of scope (likely permanently): on-tractor obstacle *avoidance* (re-planning around obstacles), cooperative multi-tractor, public-road operation.

---

## 2. Hardware delta vs. v25 baseline

The v25 BOM ([`HARDWARE_BOM.md`](../HARDWARE_BOM.md)) is **mostly sufficient**. Autonomy needs:

| Need | v25 baseline | Delta required |
|---|---|---|
| Position, ~2 m | NEO-M9N (already on tractor for telemetry) | None for path-replay / coarse waypoint follow |
| Position, ~2 cm | — | **RTK upgrade**: SparkFun ZED-F9P (~$275) on tractor + same on base or NTRIP feed |
| Heading | BNO086 IMU (already) | None; add GPS-heading fusion (dual-antenna later if needed) |
| Wheel/track odometry | — | Optional: hall-effect or magnetic on each track sprocket (~$30) |
| Steering actuator | manual joystick → hydraulic valves (already) | None; autonomy commands the same valve coils via `lora_proto` |
| Throttle | — | **New**: a way to command engine throttle (servo on the throttle linkage, or J1939 if engine ECU supports it). Currently operator-only. |
| Compute | Portenta X8 Linux (already) | Sufficient for waypoint follow / pure-pursuit. Add a Coral USB Accelerator (~$60) only if onboard vision-based obstacle detection is in scope. |
| Obstacle sense | UVC webcam (already) | Optional: time-of-flight or 2D LiDAR (~$200) for obstacle stop |
| Bumper kill switches | — | **Recommended**: front and rear contact switches into the PSR safety relay chain |

**Decision rule:** add the RTK upgrade and the throttle actuator before any field autonomy test. Everything else can wait for a real operational gap.

---

## 3. Software architecture options

The autonomy planner runs on the **base-station X8** (not the tractor). It commands the tractor over the same LoRa control path the human operator uses, just at a lower priority. Three viable stacks:

### Option A — Bare Python service on the base-station X8 (recommended start)

```
┌──────────────────────────────────────────────────────────┐
│  Base-station X8 (Linux)                                 │
│                                                          │
│  ┌──────────┐   MQTT   ┌──────────────┐   MQTT   ┌─────┐ │
│  │ web_ui   │ ───────► │ planner.py   │ ───────► │     │ │
│  │ (FastAPI)│  topics  │ (pure-pursuit│  control │ lora│ │
│  │          │ ◄─────── │  + waypoint  │  topics  │_brid│─── LoRa ──► tractor
│  │          │ telemetry│  follower)   │ ◄─────── │ -ge │
│  └──────────┘   topics └──────────────┘ telemetry └─────┘ │
│                                                          │
│            Mosquitto (loopback) is the bus               │
└──────────────────────────────────────────────────────────┘
```

- Pros: tiny dep footprint, fast iteration, no ROS learning curve, fits the existing v25 software model.
- Cons: roll-your-own coordinate transforms, time sync, message types. Limited tooling for SLAM / obstacle avoidance if scope creeps later.
- Use when: path replay + waypoint follow + coverage are the entire ambition.

### Option B — ROS 2 (Humble or later) on the base-station X8

```
┌──────────────────────────────────────────────────────────┐
│  Base-station X8 (Linux)                                 │
│                                                          │
│  ┌─────────┐  DDS  ┌──────────┐  DDS  ┌──────────┐       │
│  │ Nav2    │ ◄───► │ planner  │ ◄───► │ ros2_    │       │
│  │ stack   │       │ + map    │       │ mqtt_    │ ───► MQTT ──► lora_bridge
│  │         │       │ server   │       │ bridge   │                       ▼
│  └─────────┘       └──────────┘       └──────────┘                  LoRa frames
│                                                                          │
│  Mosquitto on loopback for the web UI (unchanged)                       │
└──────────────────────────────────────────────────────────┘             tractor
```

- Pros: Nav2 gives you costmaps, planners (NavFn, SmacPlanner), recovery behaviors, lifecycle nodes, RViz visualization. tf2 handles coordinate transforms. Reuses the archived [`ros2_bridge/`](ros2_bridge/) prototype.
- Cons: 1–2 GB of dependencies on the X8, learning curve, longer boot. The archived bridge currently uses MQTT-over-WiFi to a Pi-hosted Opta — the *bridge concept* is reusable; the transport assumption is not.
- Use when: scope grows to include real obstacle handling, sensor fusion, or integration with broader ROS ecosystem (e.g., a Gazebo simulator).

### Option C — Custom MPC / onboard planner on the tractor X8

Push the planner *down* to the tractor X8 so it can react without the LoRa round-trip. Base station only sends the goal.

- Pros: no LoRa-latency in the autonomy control loop; survives base-station outage.
- Cons: harder to debug (no easy operator-room console), splits the control logic across two boxes, the X8 on the tractor is already busy with video + telemetry services.
- Use when: LoRa latency proves to be a real autonomy problem (>500 ms RTT) — track that decision, do not pre-optimize.

**Recommendation:** start with **Option A**. Re-evaluate at the point where you need a costmap or obstacle handling.

---

## 4. Wire-protocol implications for v25

These are the things v25 must **not** foreclose. Most are already present in the design; flagging them so they survive future doc cleanups.

### 4.1 Control-source priority

[`LORA_PROTOCOL.md` § Multi-source arbitration](../LORA_PROTOCOL.md) defines: handheld > base operator > autonomy. **Keep the autonomy slot reserved**, even though no autonomy code exists yet. The arbitration code in `tractor_m7.ino`'s `pick_active_source()` already enumerates an autonomy source ID; do not remove it during cleanup.

### 4.2 New `topic_id`s reserved for autonomy

In [`base_station/lora_bridge.py` `TOPIC_BY_ID`](../base_station/lora_bridge.py), reserve (do not yet implement):

| topic_id | MQTT topic | Direction | Payload sketch |
|---|---|---|---|
| `0x40` | `lifetrac/v25/auto/plan_status` | tractor → base | active waypoint index, distance-to-goal, ETA |
| `0x41` | `lifetrac/v25/auto/cross_track_error` | tractor → base | meters off the planned path |
| `0x42` | `lifetrac/v25/auto/geofence_status` | tractor → base | inside/outside, last violation timestamp |
| `0x43` | `lifetrac/v25/auto/odometry` | tractor → base | dead-reckoned pose (for plotting) |

Reserve these so the message-id space is not consumed by ad-hoc telemetry.

### 4.3 Command opcodes reserved for autonomy

In [`LORA_PROTOCOL.md` § Command frame opcodes](../LORA_PROTOCOL.md), reserve:

| opcode | name | meaning |
|---|---|---|
| `CMD_AUTO_LOAD_PLAN` | uplink, base → tractor | tractor pulls plan ID from base over a separate request/response (LoRa is too slow for the plan blob; planner runs on base, only commands stream over LoRa) |
| `CMD_AUTO_START` | base → tractor | engage autonomy, tractor begins following stored plan |
| `CMD_AUTO_PAUSE` | base → tractor | hold position, keep autonomy armed |
| `CMD_AUTO_ABORT` | base → tractor | drop autonomy, return to operator control, log reason |
| `CMD_GEOFENCE_SET` | base → tractor | upload polygon (small enough to fit; otherwise stage on tractor X8 over a side channel) |

**Safety rule (non-negotiable):** `CMD_ESTOP` from any source preempts every autonomy opcode. Autonomy never disables `CMD_ESTOP` handling. The M4 enforces geofence and max-speed clamps independently of the planner.

### 4.4 Plan transport

LoRa is too slow for transferring a coverage plan with hundreds of waypoints. Two options:

- **Plan stays on base, only commands stream over LoRa.** The planner runs on base and sends one waypoint at a time as a `ControlFrame` extension or a dedicated low-rate topic. **Recommended.**
- **Plan staged on the tractor X8 over a side channel.** When the tractor is in the garage on Ethernet, push the plan from base to tractor via SCP/HTTP. Then autonomy operates from the local plan even if LoRa is patchy. Useful for very-large-plan operations.

Either way, the air protocol stays small and predictable.

---

## 5. Plan-file formats

For interoperability with farm-management software:

| Format | Used by | Verdict |
|---|---|---|
| **GeoJSON** | Open-source GIS, Leaflet (already in base UI) | **Primary** internal format. Easy to author, easy to render. |
| **KML** | Google Earth, many ag tools | Import/export shim. Convert to GeoJSON internally. |
| **GPX** | Hiking GPS gear, simple waypoint exports | Import shim. Limited (no polygons). |
| **ISOXML / ISO 11783-10** | Commercial ag (John Deere, Case IH, Trimble) | **Future:** import shim if interop with commercial fleet management is ever in scope. Heavy spec; do not pre-build. |
| **ROS 2 nav_msgs/Path** | Internal to a ROS-based stack (Option B) | If Option B is chosen. |

Recommendation: GeoJSON in, GeoJSON out, with optional KML / GPX shims. ISOXML deferred until a real interop need appears.

---

## 6. Why MQTT for the base-station IPC

Q: *Is MQTT the best link to automation plans?*

For the **base-station-internal IPC** between `lora_bridge`, `web_ui`, `planner`, and `logger`: **yes**, with caveats below. For the **tractor-over-the-air**: **no** — that stays LoRa frames.

| Layer | Best fit | Why |
|---|---|---|
| Base IPC: planner ↔ web_ui ↔ lora_bridge ↔ logger | **MQTT** (Mosquitto loopback) | Already in the design, decouples services, language-agnostic, free pub/sub fan-out, small footprint. |
| Base IPC if planner is ROS 2 | **ROS 2 DDS**, with an MQTT bridge for the web UI | DDS gives typed messages, QoS, time sync, tf2. The archived `ros2_bridge` is the head start. |
| Tractor ↔ base over the air | **LoRa frames** (`lora_proto`) | The bridge encodes/decodes; the air is raw `lora_proto`. Do not expose MQTT as the air protocol. |
| ≥50 Hz inside the planner | **DDS or shared memory**, not MQTT | Broker overhead is visible at high rates. The realtime control path (M7→M4→Opta) already bypasses any broker. |
| Long-term telemetry / dashboards | InfluxDB or Timescale + Grafana, fed from MQTT | Standard pattern; matches the BASE_STATION sketch. |
| Off-tractor (operator room ↔ farm-management software) | **MQTT or REST**, GeoJSON for plans | Operator posts plans to base UI → UI publishes → planner picks up. |

**Decision rule:** MQTT is the right glue *inside* the base station for things outside the realtime control loop. Anything that must be deterministic ≤50 ms (control frames, watchdog, E-stop) bypasses the broker entirely.

---

## 7. Open questions

These do not need answers to ship v25; they need answers before autonomy starts.

- Throttle actuator: aftermarket linear servo on the throttle lever, or does the engine have a J1939 ECU we can write to? (Affects HARDWARE_BOM and the `tractor_opta` Modbus map.)
- RTK base location: on the building, on the cab, or NTRIP from a nearby public CORS station?
- Field-side compute: is the base-station X8 in radio range of every field, or do we need a portable base station?
- Operator handover: when the operator takes control mid-autonomy, does autonomy resume on release (path-following with a temporary detour) or abort (operator must restart)? Default to abort for safety.
- Recording-while-driven: is the path-replay record continuous or operator-triggered (button on handheld)?
- Plan auditability: store every executed plan with timestamps + telemetry for post-hoc review (legal/safety).

---

## 8. Promotion criteria

To promote any of this to canonical scope (out of `RESEARCH-CONTROLLER/` and into `DESIGN-CONTROLLER/`), the [MASTER_PLAN.md §7 open-decisions list](../MASTER_PLAN.md#7-open-decisions-that-would-change-this-plan) must be updated and the v25 build must be field-proven (operator-driven, manual control, end-to-end LoRa, no autonomy). Order matters: do not start autonomy work before the manual-control v25 has run a real workday.

---

## 9. Vision-based obstacle stop (camera or LiDAR)

This is the lowest-bar autonomy capability *and* the one most relevant to v25 — even a non-autonomous tractor benefits from "stop if a person walks in front of you." Treated separately from path/waypoint autonomy because:

- It runs **stop-only**, never steering. No re-planning, no avoidance.
- It is a **safety overlay**, equally useful in tele-op as in autonomy.
- It can be retrofitted to v25 without touching the control-source priority logic — it injects `CMD_ESTOP`-equivalent over IPC, just like a hardware bumper switch would.

### 9.1 Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│  Tractor X8 (Linux)                                              │
│                                                                   │
│  USB cam ──► v4l2 ──► obstacle_detector.py ──► IPC flag ──┐      │
│  (Kurokesu)         │                                       │      │
│                     ├─ Coral USB Accelerator (TPU)         │      │
│                     │  running an Edge TPU person/animal    │      │
│                     │  detector (e.g. MobileNet-SSD,        │      │
│                     │  EfficientDet-Lite, YOLOv8n)          │      │
│                     │                                        ▼      │
│                     └─ publishes safety/obstacle_event   ┌────────┐│
│                                                          │ M7/M4  ││
│                                                          │ on H747││
│                                                          │ co-MCU ││
│                                                          │        ││
│                                                          │ reads  ││
│                                                          │ flag,  ││
│                                                          │ latches││
│                                                          │ E-stop ││
│                                                          └────────┘│
│                                                                   │
│  Mirrored on LoRa: CMD_OBSTACLE_STOP (0x50) → base UI shows why   │
└──────────────────────────────────────────────────────────────────┘
```

The **safety-relevant path is the IPC flag from X8 → H747** (a few ms). The LoRa frame is informational so the operator's UI can display the detection class and a thumbnail. **The H747 must not depend on receiving the LoRa frame — its own local IPC read is the authority.**

### 9.2 Hardware delta

| Item | Cost | Notes |
|---|---:|---|
| Google Coral USB Accelerator (TPU) | $60 | Inference offload from X8 CPU. Without it, MobileNet runs at ~5 fps on the X8 CPU (marginal); with it, ~30+ fps headroom. |
| Bumper kill switches (front + rear, contact / pull-cord) | $20–40 | Wired into the **PSR safety relay chain**, NOT into a software input. This is the redundant non-software safety. Vision is the *first* line; the bumper is the *unconditional* line. |
| Existing Kurokesu C2-290C front cam | $0 | Already in BOM. Sufficient for ~10–15 m person detection at 1080p. |
| (Optional) RPLIDAR A2M12 or similar 2D LiDAR | $200 | Better in fog/rain/dust where vision degrades. Defer until vision-only is proven inadequate. |

**Total cheapest path: ~$100** to add vision-based stop to v25 *as a follow-on retrofit*. Not in v25 baseline scope, but the BOM already supports the camera and the X8 has the USB ports.

### 9.3 What we ship in v25 to make this trivial later

These are the **hooks**, not the implementation:

- **`CMD_OBSTACLE_STOP` (0x50) reserved** in `LORA_PROTOCOL.md` ✅ (done in this revision).
- **Topic ID `0x50` `safety/obstacle_event` reserved** in `LORA_PROTOCOL.md` ✅ (done in this revision).
- **One D1608S input reserved** for a future bumper switch loop. Suggest input I12 (currently spare) — wired into the PSR chain so it works regardless of MCU state.
- **The H747 IPC contract from §8.10 already supports** non-blocking reads from the X8. An `obstacle_active` boolean is exactly the kind of flag that contract was designed for.
- **`pick_active_source()` already returns `SOURCE_NONE` and drops valves to neutral** when needed — the obstacle-stop path is just one more reason to enter that state. No arbitration changes needed.
- **No model weights are committed**; obstacle detection is opt-in firmware that ships in a future release.

### 9.4 Operating contract

If/when implemented:

- **Detection latency budget:** ≤ 250 ms from object entering frame to valves de-energized. (Inference ≤ 50 ms on Coral, IPC ≤ 5 ms, M4 latch ≤ 10 ms, valve-side stop per [`LATENCY_BUDGET.md`](LATENCY_BUDGET.md) items #10–#13.)
- **Detection range:** ≥ 5 m for a standing adult at 1080p with a 60° HFOV lens.
- **False-positive policy:** `confidence_pct ≥ 70%` AND object persists ≥ 3 frames before triggering. Single-frame detections are logged but do not stop. Operator can re-tune thresholds in `params_service.py`.
- **Recovery:** stop is latched. Operator must press CLEAR ESTOP on the base or handheld (same UX as any other E-stop). No auto-clear.
- **Disable for tasks where it gets in the way:** loader work in close proximity to people is the obvious case. Operator toggles a "vision stop disarmed" flag in the UI; the disarm is logged and a banner displays continuously.
- **Testing:** stand a mannequin in the path during bench bring-up. Walk a (consenting) human through the camera at decreasing distances. Log every detection event for false-positive review.

### 9.5 What this does NOT replace

- The **hardware E-stop button** on the base, the handheld, and the tractor frame.
- The **PSR safety relay** chain.
- The **bumper kill switches** (when added).
- **Operator vigilance.** This is a backstop, not a chauffeur.

---

## 10. What to do in v25 to prepare for autonomy + obstacle stop

Concrete cheap actions that keep future-release options open without expanding v25 scope. Anything more invasive than this should wait until the v25 build is field-proven.

### 10.1 Already done in this revision

- ✅ Reserved Command opcodes `0x40`–`0x44` (`CMD_AUTO_*`, `CMD_GEOFENCE_SET`) in [`LORA_PROTOCOL.md`](../LORA_PROTOCOL.md).
- ✅ Reserved Command opcode `0x50` `CMD_OBSTACLE_STOP` in [`LORA_PROTOCOL.md`](../LORA_PROTOCOL.md).
- ✅ Reserved telemetry topic IDs `0x40`–`0x43` (`auto/*`) and `0x50` (`safety/obstacle_event`) in [`LORA_PROTOCOL.md`](../LORA_PROTOCOL.md).
- ✅ Autonomy `source_id` slot kept in `pick_active_source()` (already present from earlier drafts; flagged here so it does not get cleaned up).

### 10.2 To do during v25 hardware build (small effort, high option value)

- **Wire one D1608S input as a future bumper-switch loop.** Suggest input **I12**, terminated to a screw block on the enclosure with a "FUTURE: BUMPER" label. Document in [`TRACTOR_NODE.md`](../TRACTOR_NODE.md) wiring diagram.
- **Mount the front Kurokesu camera with a 60–90° HFOV lens** rather than the narrowest available option, so a future obstacle-detector has useful field of view.
- **Leave a USB-A port free on the tractor X8** (do not consume all of them with cameras and the FT232H). The Coral USB Accelerator and the FT232H both want USB-A.
- **Reserve ~2 GB of free space on the X8 eMMC** for future model weights and a small log of obstacle events.
- **Run a 4-conductor cable from the cab to the front bumper** when assembling the harness, even if no switches are mounted. Pulling cable later through finished hydraulics is painful.

### 10.3 To do during v25 firmware bring-up (small effort, high option value)

- **Define an `IPC_OBSTACLE_FLAG` shared-memory slot** in the H747 ↔ X8 IPC region, even though no producer writes to it in v25. Reading it from the M4 should be a no-op that always returns `false`. This validates the §8.10 non-blocking IPC contract end-to-end without committing to vision.
- **Add a `safety_disarm_vision` parameter** to `params_service.py` (default `false`). No effect in v25; ensures the param plumbing exists.
- **Log every `CMD_*` opcode received**, including the reserved 0x40+ range, with "RESERVED — IGNORED IN V25" for unknown opcodes. This makes future bring-up debuggable from day one.

### 10.4 Out of scope for v25 (do NOT do)

- Do not buy the Coral, RTK GPS, throttle actuator, or LiDAR for v25. The §10.2 / §10.3 changes are sufficient hooks.
- Do not implement `CMD_AUTO_*` or `CMD_OBSTACLE_STOP` handlers beyond the "log and ignore" stub above.
- Do not commit any model weights or vision pipeline code to `firmware/tractor_x8/`.
- Do not expand the §5 bring-up gate or §8.15 field-test gate to include autonomy criteria.

### 10.5 Promotion path

When the v25 build has been field-proven (per [MASTER_PLAN.md §8.15](../MASTER_PLAN.md#815-field-test-gate--500-m-line-of-sight-1-frame-loss-10-min)) and an operational need for autonomy or obstacle stop is identified, the promotion sequence is:

1. Open a §7 reversal in [MASTER_PLAN.md](../MASTER_PLAN.md) for the specific capability (autonomy *or* obstacle stop, not both at once).
2. Move the relevant subsection of this document into a new `DESIGN-CONTROLLER/AUTONOMY.md` or `DESIGN-CONTROLLER/SAFETY_VISION.md`.
3. Add the hardware delta from §2 / §9.2 to [`HARDWARE_BOM.md`](../HARDWARE_BOM.md).
4. Implement the previously-reserved opcodes/topic-ids; the protocol does not need a version bump because the slots were pre-allocated.
5. Re-run the §5 bring-up gate with the new capability before claiming readiness.

This sequence is what "preparing for a future release" actually means in this repo: **allocate the namespace, wire the latent hardware paths, and write zero functional code now.**
