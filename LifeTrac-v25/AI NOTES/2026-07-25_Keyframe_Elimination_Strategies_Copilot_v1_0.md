# Keyframe Elimination Strategies for the Tile-Delta Image Pipeline
## Creative ways to avoid the required use of keyframes

**Document version:** v1.1 (RS-6.0 transport revision, 2026-08-02 — see addendum below)
**Date:** 2026-07-25
**Author:** GitHub Copilot (assistant-owned document)
**Scope:** `camera_service.py` → LoRa → `image_rx_daemon.py` → web canvas
**Companion docs:**
- `DESIGN-CONTROLLER/IMAGE_PIPELINE_METHODS.md` (Method A/B/C tile selection)
- `2026-07-25_LoRa_Image_Pipeline_Future_Work_Roadmap_Copilot_v1_0.md` (§3.1, §4.1)
- `CODE REVIEWS/2026-07-23_LoRa_Comm_and_Image_TX_RX_Optimization_Review_Copilot_v1_0.md` (P1/P6)

---

## Addendum v1.1 — RS-6.0 LoRa-only transport revision (2026-08-02)

Per the 2026-07-25 architecture rule (**base→tractor strictly over
LoRa**), every back-channel this document sketched as an MQTT topic is
respecified as a **0xFB command-frame opcode over the radio**, admitted
through the same QoS/slot machinery as all command traffic. Concretely:

1. **K-B is substantially SHIPPED.** F10 (merged 2026-08-01, verified on
   air — `bench-evidence/F10_tile_stale_acceptance_2026-08-01`) is
   K-B's staleness-driven NACK in all but name: `CMD_OP_TILE_STALE`
   (`0xFB` opcode `0x6C`, `u16le base_seq_ref + 12 B bitmap`, 14 B body
   ≈ 15.4 ms at DTS) reports stale tiles from the base's
   `arrived_ms` clock; the tractor ORs the marks into the Method-C
   age-escalation exactly as §K-B's "TX ORs the requested bitmap"
   prescribed. Level-triggered single-copy, no retry machinery. The
   "field note" about the LAN back-channel is obsolete — it rides LoRa
   today. F11 (2026-08-02) additionally removed the per-seq-gap
   keyframe request, so gap repair is now K-B-style tile repair by
   default. Remaining delta to full K-B: an explicit `req_tiles(all)`
   connect-time form (see K-G below).
2. **K-C hash beacon = a DOWNLINK payload type**, not a topic: specify
   as a new tile-delta stream frame kind (or a `0xFB` downlink-adjacent
   frame) carrying the 96×CRC-8 vector — one ~102 B fragment every M
   seconds inside the image budget. The mismatch response reuses `0x6C`
   with the diffed bitmap (detector K-C → effector K-B, unchanged).
3. **K-G connect-time sync**: the cold-start `req_keyframe` retained by
   F10/F11 (`0xFB` opcode `0x60`) is the interim K-G trigger; the K1
   target replaces its response with paced P-frame chunks (`req_tiles
   (all)` semantics — proposed as `0x6C` with an all-ones bitmap +
   a reserved flag, avoiding a new opcode).
4. **Uplink budget accounting**: `0x6C` costs one command slot per
   report tick (default 3 s, level-triggered so silence is free); at
   DTS the 24 B on-air frame ≈ 15.4 ms — under 0.6 % duty even
   reporting continuously. All uplink forms above must stay within the
   command-plane admission (same pump/idle-drain paths as today).
5. **Numbering caution**: the 0xFB `CMD_OP_*` namespace collides with
   the design-era `frame_type 0x30` table on `0x60`/`0x62` — see
   `LORA_PROTOCOL.md § Shipped 0xFB bench command set`. K1/K2 work must
   reconcile deliberately.
6. **Exit-metric instrumentation now exists** (RS-6.1, 2026-08-02):
   per-tile `age_ms` rides `/ws/state`, and the archivable aggregate
   (`p50/p95/max age, missing, stale count`) publishes on
   `lifetrac/v25/status/tile_age` every scan tick — a healthy link's
   p95 age is the sweep-rotation measurement K1/K2 exits score against.

---

## 1. Why keyframes exist here — and why they are *weaker* than they look

The wire format ([frame_format.py](../DESIGN-CONTROLLER/base_station/image_pipeline/frame_format.py))
settles the most important architectural question up front:

> **Every tile blob is absolutely coded.** A `TileDeltaFrame` "P-frame" is
> a *set of self-contained tiles* (WebP/BTC4/mono-G4 per tile) plus a
> bitmap of which grid cells they replace. Nothing on the wire is coded
> as a diff against previous *pixel content*.

Consequently, losing a P-frame **never corrupts** receiver state — it only
leaves some tiles **stale**. The decoder cannot desynchronize the way a
real video codec (H.264 etc.) does. Keyframes in this system therefore do
only two jobs:

| Job | What actually requires it |
|---|---|
| **J1 — Initial acquisition** | A receiver that just connected has no canvas; it needs every tile once. |
| **J2 — Eventual consistency** | Tiles whose updates were lost (reassembler eviction drops whole frames) must eventually be re-sent. |

Neither job requires a *periodic, full-canvas, scheduled* keyframe. That
design is a blunt instrument: 96 tiles × ~100–200 B ≈ 10–20 KB in one
burst (~6–12 s of exclusive airtime at FHSS rates, ~5–10 s of it wasted
re-sending tiles the receiver already has), and when the burst itself
loses a fragment, the whole frame evicts and the canvas stalls for
another `KEYFRAME_PERIOD_S` (code default 10 s; 60 s in the review-era
unit files — either way the worst perceived-latency event in the system).

**Goal:** meet J1 and J2 without ever scheduling a full-canvas frame.

## 2. What the codebase already provides (¾ of the machinery exists)

- **Method C rotating fair-share sweep** (`SWEEP_STEP`, `tile_last_seq`):
  every P-frame force-includes the N longest-unshipped tiles. At
  SWEEP_STEP=2 / 1 fps, a static 96-tile canvas converges in ~48 s with
  *zero* keyframes. This **is** gradual intra refresh (GDR from the video
  world) — already shipping, currently used *alongside* keyframes rather
  than instead of them.
- **`req_keyframe` back-channel** (`lifetrac/v25/cmd/req_keyframe`, RX
  self-heal on reassembly failure + manual web-UI button) — proves a
  receiver→sender control path exists end-to-end.
- **`CMD_LINK_PROFILE` / `link_stats`** — precedent for receiver-driven
  *parameter* adaptation, reusable for receiver-driven *repair*.
- **`add_parity_fragments`** (0xFD redundancy wrapper, tested, unused) —
  fragment-level FEC ready to wire.
- **Per-frame `base_seq`** — RX can detect skipped frames (seq gaps) even
  when it cannot know which tiles the lost frame carried.

## 3. The menu

Ordered roughly from "config change" to "research project".

### K-A. Keyframeless steady state (promote the sweep, retire the schedule)
Set `KEYFRAME_PERIOD_S = ∞`; make the Method C sweep the *sole* J2
mechanism, with an **adaptive SWEEP_STEP**: scale the number of forced
refresh tiles with spare frame budget (bytes left under
`LIFETRAC_FRAGMENT_BUDGET` after motion tiles are packed). Still scenes
refresh fast (whole budget = sweep); busy scenes degrade gracefully
(motion first, sweep trickles).
- **Cost:** ~20 lines in `camera_service.py`; no wire change; no RX change.
- **Bound:** worst-case tile staleness = `n_tiles / SWEEP_STEP_eff` frames —
  *deterministic*, unlike today's "stall until next keyframe survives".
- **Risk:** none for J2. J1 unhandled → pair with K-G.

### K-B. Receiver-driven per-tile NACK ("give me these 5 tiles")
Generalize `req_keyframe` into `req_tiles` carrying a 12-byte tile bitmap.
RX detects `base_seq` gaps; since it cannot know *which* tiles a lost
frame carried, it requests tiles by **staleness**: any tile older than
the sweep guarantee, or all-tiles (bitmap of 1s) at connect. TX ORs the
requested bitmap into the next P-frame's changed set (the Method C
machinery already merges forced tiles).
- **Cost:** small; one new topic + bitmap merge; reuses everything.
- **Payoff:** turns the 60 s worst case into ~1 frame period; repairs cost
  exactly the lost tiles, not 96.
- **Field note:** on bench, the back-channel rides the LAN. In the field
  it must ride the LoRa control uplink (`control_sf7` profile) — a 14-byte
  payload, trivially affordable, but the uplink path must exist (it is
  already required for driving commands, so no *new* dependency).

### K-C. Canvas-hash beacon (stateless sync, no receiver bookkeeping trust)
TX appends a **tile-generation digest** to the stream every M seconds: a
96 × 1 B vector (CRC-8 of each tile's last-shipped blob), ~102 B — one
fragment. RX hashes its own canvas tiles the same way, diffs, and
`req_tiles` the mismatches. This catches *every* divergence source
(losses the seq-gap heuristic missed, RX restarts, bugs) with provable
convergence, and costs ~1 fragment per M seconds.
- Think of it as a 1-level Merkle sync; a 2-level variant (1 root byte
  per frame header, full vector on mismatch) costs one byte per frame.
- **Best paired with K-B** (it is the detector; K-B is the effector).

### K-D. TX-side receiver model (ACK'd tile generations)
RX periodically acks `(base_seq, bitmap-of-applied-tiles)`; TX keeps a
"receiver has generation g of tile t" model and schedules re-sends only
for un-acked deltas. This is full ARQ semantics per tile.
- **Payoff:** zero redundant refresh bytes — theoretical optimum for J2.
- **Cost:** the highest of the menu — new state machine on TX, ack
  cadence tuning, handling multiple receivers (fleet case) forces either
  per-receiver models or degrade-to-K-C behaviour anyway.
- **Verdict:** hold for the field-PER data; K-B/K-C likely suffice.

### K-E. Temporal FEC over tiles (repair without a back-channel)
Every k-th frame, send one **XOR heal tile**: the XOR of the last k
shipped tile blobs (padded), tagged with their (frame, index) list. A
receiver missing exactly one member reconstructs it locally — no uplink
round-trip. Generalizes to fountain/Raptor-lite streams where any n-of-m
symbols rebuild the canvas (elegant: there is *no distinguished keyframe
at all*, initial sync and repair are the same operation).
- **Cost:** medium; new codec id (escape hatch `CODEC` 5–14 reserved) and
  RX reconstruction cache.
- **When it wins:** links with a broken/expensive uplink, or multicast to
  many receivers (one heal stream fixes different losses at different
  receivers simultaneously — the fleet/spectator case).

### K-F. Idle-airtime opportunistic refresh (make J2 free)
Send sweep/refresh tiles **only when the QoS bucket is otherwise idle**
(TX daemon knows its own budget utilization; runs 29/30 showed 25–51 %
idle even under load). Refresh traffic then costs *zero* motion-tile
latency. Combine with K-A's adaptive step: `SWEEP_STEP_eff =
f(idle_budget)`.

### K-G. Connect-time-only sync (J1 without scheduled keyframes)
The only time a full canvas is genuinely required is receiver start.
RX daemon (or web_ui session start) fires one `req_tiles(all)`; TX
streams the 96 tiles as *paced P-frames* (12-fragment budget chunks,
interleaved with live motion tiles) rather than one monolithic I-frame —
so even initial sync never creates the loss-amplifying 10–20 KB burst.
After that: K-A/B/C forever.

### K-H. Sidestep the problem (change what a "frame" is)
- **ROI-live + static mosaic:** keep only motion regions live; transmit
  the static background once as a slow build (the Phase-2 review's
  strip/mosaic concept). The "keyframe" becomes a one-time site survey.
- **Semantic overlay:** ship detected objects/poses as vectors over a
  stale photo background — keyframes become irrelevant to operation
  (the operator needs *where the trench edge is now*, not fresh gravel
  pixels). Long-term, pairs with the autonomy roadmap sensors.
- Listed for completeness; both change product behaviour, not just
  protocol.

## 4. Interactions with the LoRa layer (the constraints that matter)

1. **Loss granularity is a whole `TileDeltaFrame`** (reassembler eviction),
   so smaller frames lose less per event. K-A/K-G's paced small frames
   inherently reduce blast radius; conversely a scheme that grows frames
   (batching per §2.2 of the roadmap) raises per-loss tile count — tune
   `SWEEP_STEP`/NACK cadence together with batching.
2. **Uplink asymmetry:** K-B/K-C/K-D need tractor-bound bytes. Budget:
   a 14 B `req_tiles` at control_sf7 ≈ 25 ms ToA — negligible, but the
   *scheduler* must admit it during image saturation (mailbox handles it;
   verified pattern from driving-command preemption reviews).
3. **Profile ladder:** at DTS 1.76 KB/s the whole question softens (a full
   96-tile refresh ≈ 6–10 s of background trickle); at FHSS 699 B/s and
   worse field rates, K-B/K-C dominate. Design for the FHSS budget; treat
   DTS as headroom.
4. **`base_seq` is u8** — seq-gap detection wraps at 256 frames; the K-C
   hash beacon covers the wrap blind spot.

## 5. Recommended composition and phasing

**Phase K1 — "no scheduled keyframes" (config + ~40 lines):**
K-A (adaptive sweep, `KEYFRAME_PERIOD_S=∞`) + K-G (connect-time paced
sync via existing `req_keyframe`, re-labelled). Keyframes stop existing
as a scheduled event. Exit test: kill RX mid-stream, restart it, canvas
fully painted ≤ 15 s at FHSS rates with zero I-frames on the wire.

**Phase K2 — targeted repair (the real win):**
K-C hash beacon (detector) + K-B `req_tiles` (effector) + K-F idle-airtime
scheduling. Exit test: inject 20 % frame loss (harness PER knob or RF
attenuator); P95 per-tile staleness ≤ 2× the loss-free value; repair
bytes ≤ 1.2× (lost tiles × tile size).

**Phase K3 — only if field data demands:**
K-E XOR-heal for uplink-poor scenarios / multi-receiver; K-D receiver
model if repair-byte overhead measurably matters at fleet scale.

**Measurement to add first** (prerequisite for all exit tests): per-tile
staleness histogram on RX (`age_since_applied` per grid cell, published on
`link_stats`) — today the system cannot even *see* tile staleness, only
frame counts.

## 6. Summary table

| Scheme | J1 | J2 | Wire change | Uplink needed | Effort | Verdict |
|---|---|---|---|---|---|---|
| K-A adaptive sweep | – | ✓ | none | no | XS | do now |
| K-G connect sync | ✓ | – | none | yes (once) | S | do now |
| K-B `req_tiles` NACK | ✓ | ✓ | new topic | yes | S | Phase K2 |
| K-C hash beacon | – | ✓✓ | +1 payload type | yes | S | Phase K2 |
| K-F idle-airtime refresh | – | ✓ | none | no | S | Phase K2 |
| K-E XOR-heal / fountain | ✓ | ✓ | new codec id | **no** | M | K3 / multicast |
| K-D receiver model | – | ✓✓ | acks | yes (cadence) | L | hold |
| K-H mosaic/semantic | ✓ | n/a | product-level | – | L+ | separate track |

The punchline: because tiles are already absolute, this system never
*needed* keyframes for correctness — only for coverage. Method C's sweep
already proves coverage works without them; the remaining work is
promoting that from a background nicety to the designed mechanism, and
giving the receiver a voice (`req_tiles`) so repair is targeted instead
of periodic and total.
