# PATH_PLANNING — research notes for a future base-station path planning system

**Status:** future-enhancement research. **Not on the build path.** Filed here so when we get to autonomy (Phase 10+ in [../../TODO.md § Stretch goals](../../TODO.md#stretch-goals-phase-10)) we already have the survey done and a sane shortlist of libraries to start from.

The premise: once the [base station web UI](../../BASE_STATION.md) is up and running the basics ([RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/](../EXAMPLE_CODE/base_station/)), the natural next step is a **map page** that lets the operator (a) see the tractor's live position and breadcrumb trail, and (b) draw waypoints / fields / no-go zones for the tractor to follow autonomously.

This document covers:

1. What "path planning" actually means for a small open-source tractor.
2. The minimum viable map page we'd build first.
3. A survey of existing open-source projects we should study before writing anything.
4. A staged build plan that integrates with the v25 controller architecture.
5. Open questions to resolve before we commit to any of it.

---

## 1. What "path planning" means here

There is a *spectrum* of automation, and we should be honest about which rung we're targeting at each phase.

| Level | Description | What the system does | What the operator does |
|---|---|---|---|
| **L0 — track + log** | Tractor's GPS position is shown on a map; trail is recorded. No autonomy. | Display + record. | Drives manually; reviews tracks afterward. |
| **L1 — waypoint go-to** | Operator clicks a point; tractor drives to it in a straight line. | Compute heading-to-waypoint, command drive valves. Stops at a radius. | Sets one waypoint; supervises. |
| **L2 — waypoint sequence** | Operator drops a list of waypoints; tractor visits them in order. | Sequence + handover between segments. | Plans the route once, supervises. |
| **L3 — coverage / boustrophedon** | Operator draws a polygon ("this field"); tractor computes a back-and-forth pattern with implement on/off zones and headland turns. | Generates the actual mowing/tilling pattern from a polygon + implement width. | Draws the field outline, picks implement width and overlap. |
| **L4 — obstacle-aware coverage** | L3 + onboard perception (camera/LiDAR) avoids dynamic obstacles. | All of L3 + reactive replanning. | Watches; intervenes if needed. |
| **L5 — multi-machine, mission-level** | Fleet management; one base orchestrates several tractors. | Out of scope for any near-term LifeTrac discussion. | — |

**Realistic LifeTrac v25 target:** L0 → L1 → L2, with L3 as a stretch once the rest is solid. **L4+ is out of scope for v25** because (a) we don't have the onboard perception stack, (b) the LoRa link budget can't carry the supervisory bandwidth a true L4 system needs, and (c) the legal/safety bar gets dramatically higher (operator must remain in line-of-sight per the project's overall safety posture).

---

## 2. Minimum viable map page (the "L0/L1" first cut)

What the **first** version on the base-station web UI should look like:

```
┌────────────────────────────────────────────────────────────────┐
│  LifeTrac v25 · Map                              SOURCE: BASE  │
│ ────────────────────────────────────────────────────────────── │
│  ┌──────────────────────────────────────┐  ┌────────────────┐  │
│  │                                       │  │ Waypoints      │  │
│  │   [ Leaflet map, tile cache local ]   │  │ ──────────────│  │
│  │                                       │  │ 1. shop door   │  │
│  │   • tractor live (heading arrow)      │  │ 2. compost pile│  │
│  │   • breadcrumb trail (last 1 h)       │  │ 3. east gate   │  │
│  │   • clickable waypoints with index    │  │                │  │
│  │   • drawn polygons (later, L3)        │  │ [+ add here]   │  │
│  │                                       │  │ [send to       │  │
│  │                                       │  │  tractor]      │  │
│  └──────────────────────────────────────┘  └────────────────┘  │
│  Status: GPS fix 8 sats · 0.7 m HDOP · trail saved             │
└────────────────────────────────────────────────────────────────┘
```

Functional requirements for v1:

- Map tiles from **OpenStreetMap, cached locally on the X8** so the page works without internet (most farms have intermittent connectivity). Pre-download the tile pyramid covering the property at install time.
- Tractor position: subscribe to `lifetrac/v25/telemetry/gps` (already in [LORA_PROTOCOL.md](../../LORA_PROTOCOL.md) topic table at 5 Hz), render a heading arrow.
- Breadcrumb trail: keep the last N points client-side; persist a longer trail in SQLite/InfluxDB on the X8.
- Waypoints: click the map to add; drag to reorder; named labels; save in SQLite. **No autonomy yet** — the "send to tractor" button does nothing in v1; it just exports the list.
- Manual driving works exactly as today; the map is read-only on the control side.

That's a 2–3 week build. After that, autonomy gets layered on.

---

## 3. Existing projects to study

Sorted into the categories that match where each one would slot into our stack.

### Map/UI library (in the browser)

- **[Leaflet](https://leafletjs.com/)** — small, no-dependency, great touch support, plugin ecosystem, OSM tile support, plays nice with FastAPI/static-served pages. **Default pick**, fits our existing [base_station/web/](../EXAMPLE_CODE/base_station/web/) approach.
- **[MapLibre GL JS](https://maplibre.org/)** — vector tiles, smoother panning, more JS weight. Overkill for our use case unless we go to vector basemaps.
- **[OpenLayers](https://openlayers.org/)** — feature-complete but heavier; only worth it if we need WMS overlays from agricultural databases.

### Agricultural autonomy / coverage planning (the prior art)

- **[FarmBot Web App](https://github.com/FarmBot/Farmbot-Web-App)** — open-source full-stack farm-control website with map UI for plant/cell-level operations. Different scale (raised bed, not field), but the **architecture pattern** (Rails + React + WebSocket-to-MQTT-to-MCU) is exactly the stack we're copying in spirit. Worth reading their `Garden Map` and `MoveAbsoluteForm` components for UX inspiration.
- **[AgOpenGPS](https://github.com/farmerbriantee/AgOpenGPS)** — the de facto open-source ag autosteer + section-control platform. Windows-first, .NET, but the **field/boundary data structures** (`.fld` files, AB-line, U-turn / keyhole / Dubins-path turn types) are well-documented and worth borrowing as a schema. Also has years of community-vetted boustrophedon-pattern code in [`SourceCode/GPS/Forms/Position/FormGPS_Position.cs`](https://github.com/farmerbriantee/AgOpenGPS/tree/master/SourceCode/GPS).
- **[QtAgOpenGPS](https://github.com/torriem/QtAgOpenGPS)** — Linux/Qt port; Linux portability proves the algorithms aren't Windows-bound.
- **[Field Robot Event](https://www.fieldrobot.com/event/)** — academic competitions where teams publish small-tractor / small-robot path planners; many have GitHub mirrors. Good source of compact ROS 2 / Python implementations.
- **[Bonsai / Open Agriculture Initiative archive](https://github.com/OpenAgInitiative)** — defunct but archived; some path-planning notebooks worth sifting.

### Mobile-robot navigation stacks (general purpose, more capable than we need)

- **[ROS 2 Nav2](https://github.com/ros-navigation/navigation2)** — full nav stack: global planner (NavFn, SmacPlanner), local planners (DWB, MPPI, RPP), behavior trees, recoveries. Already on our [Phase 10 ROS 2 bridge](../ros2_bridge/) horizon — would be the natural home for any serious autonomy. **Heavy** — runs on the X8 Linux side, not on the M7.
- **[OpenPlanner / Autoware](https://github.com/autowarefoundation/autoware)** — full self-driving stack. Way too much, but the *waypoint graph* representation (`.osm` lanelets) is a clean schema for ag headlands.
- **[MoveBase Flex](https://github.com/magazino/move_base_flex)** — modular planner runner, lighter than full Nav2. ROS 1 origin, ROS 2 port exists.
- **[OMPL](https://ompl.kavrakilab.org/)** — sampling-based motion planning library (C++ and Python). Useful if we ever want RRT* / PRM for "find a path through an orchard with obstacles" style problems.

### Coverage path planners (the L3 piece specifically)

- **[Fields2Cover](https://github.com/Fields2Cover/Fields2Cover)** — purpose-built C++/Python library for **agricultural coverage** with headland turns, obstacle-avoidance polygons, multiple swath generators, Dubins/Reeds-Shepp turn primitives. **Strongest single library for L3.** Apache-2.0. Active maintenance. Built specifically for this use case.
- **[OpenCV `findContours` + custom boustrophedon`](https://docs.opencv.org/)** — for very simple rectangular fields, a 200-line Python script using OpenCV polygon ops + Shapely is enough. Worth the trade-off if we want to avoid pulling in Fields2Cover's C++ build.
- **[CV-AreaCoverage](https://github.com/ethz-asl/cv-area-coverage)** (ETH Zurich) — academic implementation of Bouligand-style decomposition for non-convex polygons.

### GPS / RTK hardware path

- The Portenta Max Carrier already has a **u-blox SAM-M8Q** (single-frequency, 2.5 m CEP) per [HARDWARE_BOM.md](../../HARDWARE_BOM.md). Good enough for L0/L1, marginal for L2, *not* enough for L3 row alignment in row crops.
- For L3 we'd want **RTK** (~2 cm). Options:
  - **ArduSimple simpleRTK2B** (~$300 rover) + a community NTRIP base or our own [SparkFun RTK Facet](https://www.sparkfun.com/products/19984) (~$500 for a base).
  - **u-blox ZED-F9P** (the ASIC inside both of the above).
  - Free correction streams: [RTK2Go](http://rtk2go.com/) community network where coverage exists; otherwise a self-hosted base on a known surveyed point near the property.
- These all output NMEA / UBX over UART → the same telemetry path we already have for the SAM-M8Q. Software cost to upgrade is essentially zero; hardware cost is the barrier.

### Storage / sync

- **[GeoJSON](https://geojson.org/)** for waypoint and field-boundary interchange — universal, human-readable, supported by every GIS tool.
- **[SpatiaLite](https://www.gaia-gis.it/fossil/libspatialite)** or **[GeoPackage](https://www.geopackage.org/)** if we want to store fields/waypoints with spatial indexing on the X8 SQLite.
- **[Mapnik](https://mapnik.org/)** if we ever want to render our own tiles from OSM PBF on the X8 instead of using pre-baked OSM tiles.

---

## 4. Staged build plan (when this becomes priority)

| Phase | Deliverable | Depends on | Effort |
|---|---|---|---|
| **P-1 · Map page (L0)** | Leaflet page at `/map`, live tractor position, breadcrumb trail, local OSM tile cache. Read-only. | Phase 4 (LoRa link up + GPS telemetry flowing) | 2 weeks |
| **P-2 · Waypoint editor** | Click-to-add, drag-to-reorder, named, persist in SQLite. Export GeoJSON. **Still no autonomy** — operator drives manually with the map open. | P-1 | 1 week |
| **P-3 · Single-waypoint go-to (L1)** | "Send to tractor" pushes one waypoint over LoRa via a new `CMD_GOTO` command frame. Tractor's M7 runs a heading-controller, drives toward the point, stops at a configurable radius. **`CMD_ESTOP` + base operator presence required throughout.** SOURCE_AUTONOMY appears in the arbitration table. | P-2 + tractor-side autonomy MCU code | 4 weeks |
| **P-4 · Waypoint sequence (L2)** | Multi-waypoint missions. Tractor reports current waypoint index in telemetry; UI shows progress. Skip / pause / resume / cancel. | P-3 | 2 weeks |
| **P-5 · RTK upgrade (optional)** | Swap SAM-M8Q for ZED-F9P on the tractor; add an RTK base (or RTK2Go subscription). NMEA path unchanged. | P-4 (validates value before spending hardware money) | 1 week firmware + hardware |
| **P-6 · Field boundary editor** | Draw polygon on map; persist; import/export GeoJSON / .fld (AgOpenGPS) / KML. | P-2 | 2 weeks |
| **P-7 · Coverage planner (L3)** | Integrate **Fields2Cover** in the X8 Linux side. UI inputs: implement width, overlap, turn radius, entry direction. Output: ordered waypoint list pushed via P-4 mechanism. | P-4 + P-6 + RTK strongly recommended | 6–8 weeks |
| **P-8 · Section control** | If/when we add a multi-section implement, switch implement on/off based on whether the bucket/spray boom is over an already-covered area. Same data structure as AgOpenGPS section coverage map. | P-7 | 4 weeks |

**Hard prerequisites before starting any of this:**

1. The LoRa control + telemetry chain ([RESEARCH-CONTROLLER/EXAMPLE_CODE/](../EXAMPLE_CODE/)) must be solid in the field, including HIL tests and a validated arbitration loop with `SOURCE_AUTONOMY` defined but not used.
2. A documented and field-tested E-stop path that bypasses autonomy entirely. The base-station web UI must keep its current "always-active" E-STOP button, and the tractor-mounted physical E-stop must remain authoritative regardless of what autonomy is doing.
3. An operator-presence requirement: **autonomy mode requires the base-station operator to keep a "dead-man" button held**, or autonomy times out within a configurable window (suggest 5 s). This is the same pattern marine autopilots and FarmBot use.

---

## 5. Open questions

These are the things we should *not* decide in advance — they need real field experience first:

1. **Where does the planner *run*?** Three options:
   - On the X8 (Linux side), pushing waypoints to the M7. Simpler. The X8 might reboot.
   - On the M7. Hard real-time, no Linux, but limited memory and no easy access to map/spline libraries.
   - On the operator's laptop browser, pushing waypoints over WebSocket. Fragile (browser tab close = autonomy dies) but lowest barrier to iterate. **Probably the right starting point for L1/L2; move to X8 for L3.**

2. **Coordinate frame on the wire.** Do we send WGS-84 lat/lon over LoRa (16 bytes for two doubles), or convert to a local ENU frame relative to a known anchor (8 bytes for two int32 mm-precision)? **Lean toward local ENU** — saves bytes, keeps math simple on the M7, anchor only needs to be re-set if we move the property.

3. **Heading source.** The SAM-M8Q gives course-over-ground (only valid above ~0.5 m/s) but no true heading at standstill. We'd want either (a) a magnetometer (cheap, but cab steel will throw it off), (b) dual-antenna GPS (expensive), or (c) wheel-odometry + IMU dead-reckoning between GPS updates.

4. **What happens when the LoRa link drops mid-mission?** Options:
   - Tractor pauses at the next waypoint (fail-safe but poor UX).
   - Tractor continues on the current segment for a configurable distance, then stops if no link (compromise).
   - Tractor returns to a "home" waypoint (potentially worse — driving without supervision).
   We probably want option 2, but this needs an explicit policy in [LORA_PROTOCOL.md](../../LORA_PROTOCOL.md).

5. **Legal posture.** Most US states require an operator-in-attendance for self-propelled ag equipment over a certain weight. Even L2 might need a licensed operator within line-of-sight. Should be checked before any field demo.

6. **Whether to wrap Nav2 instead of building bespoke.** Wrapping ROS 2 Nav2 (via the planned [ros2_bridge/](../ros2_bridge/)) gets us a *huge* amount of mature code for free, at the cost of pulling the entire ROS 2 dependency stack onto the X8. Probably worth it once we get past P-4.

---

## 6. References

- **AgOpenGPS** — <https://github.com/farmerbriantee/AgOpenGPS> · [forum](https://discourse.agopengps.com/)
- **Fields2Cover** — <https://github.com/Fields2Cover/Fields2Cover> · [paper (IROS 2023)](https://arxiv.org/abs/2210.07838)
- **FarmBot Web App** — <https://github.com/FarmBot/Farmbot-Web-App>
- **ROS 2 Nav2** — <https://github.com/ros-navigation/navigation2> · [docs](https://docs.nav2.org/)
- **OMPL** — <https://ompl.kavrakilab.org/>
- **u-blox ZED-F9P** — <https://www.u-blox.com/en/product/zed-f9p-module>
- **ArduSimple simpleRTK2B** — <https://www.ardusimple.com/simplertk2b/>
- **RTK2Go** community NTRIP — <http://rtk2go.com/>
- **Leaflet** — <https://leafletjs.com/>
- **OpenStreetMap tile usage policy** (matters once we start caching) — <https://operations.osmfoundation.org/policies/tiles/>
- **GeoJSON** — <https://geojson.org/>
- v25 base-station UI design context — [../../BASE_STATION.md](../../BASE_STATION.md)
- v25 LoRa wire format (where `CMD_GOTO` would land) — [../../LORA_PROTOCOL.md](../../LORA_PROTOCOL.md)
- Phase 10 stretch goals (this document feeds in there) — [../../TODO.md](../../TODO.md)
- Sibling research folder — [../VIDEO_COMPRESSION/README.md](../VIDEO_COMPRESSION/README.md)
- Sibling research folder — [../EXAMPLE_CODE/README.md](../EXAMPLE_CODE/README.md)
