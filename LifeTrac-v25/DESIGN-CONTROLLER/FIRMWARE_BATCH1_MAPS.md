# Firmware Batch 1 — verified implementation maps (F6-F9)

Generated 2026-07-30 against commit 54ac9797 by a map-then-verify agent pass:
one mapper per item, then an independent verifier that re-checked every
file:line and safety claim against the tree and rewrote the map with
corrections applied. Each section is the VERIFIED map with the verifier's
correction report in front of it. At least one correction is load-bearing:
F6/C1 shows the naive lock-out does not close its own recovery loop on a
duplex node and needs a grid-adoption flag.

These are maps, not code. Implementation still pays the bench flash loop,
which is fragile (RS-5.2: tractor gpio163 dead, 7-10 s SWD path).


---

# ITEM F6-epoch-drift-lockout

# VERIFICATION REPORT

## Claims verified correct (evidence)

Every file:line claim in the map was checked against the tree; the following all hold exactly as cited:

- `radio/sx1276_rx.c:174-176` consider_remote call; unconditional `scan_feed_frame(true)` at `:217`; anchor block gated on SNAPPED/ALIGNED at `:196-212`; payload-delivered-on-REJECTED comment at `:166-173`; `_Static_assert` at `:30-31`; LOCKED→BEGIN_SCAN clock-reset edge at `:640-644` (resets only `sx1276_fhss_clock_reset()` + follow flag — scheduler epoch untouched, confirmed by reading `scan_dispatch_action` `:449-494`, which resets only the walker).
- `radio/sx1276_fhss.c`: `s_fhss` at `:45`, drift gate `:232-236`, `SNAPPED` snap `:242-244`, `consider_remote` spans `:210-245`, static asserts `:14-18`, no state mutation on any REJECTED path.
- `include/sx1276_fhss.h`: decision enum `:118-134`, `SNAP_MAX_EPOCH_DRIFT 1U` at `:141`, contract comment `:143-165`, and the **stale mirror comment at `:36-39` — confirmed stale**: `bench/host_proto/fhss_scheduler.py` does not exist (directory listing shows only `fhss_chantab.py`; golden vectors are C-only).
- `radio/sx1276_rx_scan_policy.c`: SCANNING+FRAME_VALID→LOCK `:54-59`; LOCKED+FRAME_VALID→REANCHOR `:122-127`; LOCK_LOSS demotion `:137-143`; LOCKED+INVALID→HOLD `:146-150`. `SX1276_RX_SCAN_LOCK_LOSS_MS 2000U` at `include/sx1276_rx_scan_policy.h:77`.
- `radio/sx1276_fhss_clock.c`: `s_clk` struct `:8-14`, wrap-safe idiom `:35`. "Anchoring discipline" comment is at `include/sx1276_fhss_clock.h:34-42` (the .h, as the map's §3.3 places it); "converges on the busier node" intent at `:39-40`.
- `radio/sx1276_tx.c`: `tx_now_ms = platform_now_ms()` at `:206`, lazy first-anchor `:207-214`, abs_now snap `:215-221`, `slot_offset` sample `:222-224`. The floor-identity math for the proposed refresh checks out: with `d = t - anchor = q*SLOT + r`, re-anchoring at `(t - r, anchor_abs + q)` leaves `abs_slot(u)` and `in_slot_ms(u)` bit-identical for all `u` (same 49.7-day caveat as existing code), and the `:222` `in_slot_ms` read after the refresh returns the same `r`.
- `include/sx1276_rx.h`: DIM `5U` at `:48`, "(0..4)" comment at `:38`; `radio/sx1276_rx_counters.c` is fully DIM-driven (record guard `:23`, loops `:36,:42`) — no code change needed.
- Makefile (at `firmware/murata_l072/Makefile`): `check` at `:246` has exactly **25** targets; `check-fhss-scheduler` `:413-421` compiles real `sx1276_fhss.c`; `check-tx-len-guard` `:619-634` compiles real `sx1276_tx.c` + `sx1276_fhss.c` + `sx1276_fhss_clock.c` under `-DLIFETRAC_FHSS_TX_ROUTED`; `check-rx-counters` `:685-692`; `check-fhss-clock` `:814-821`. **No bench target compiles `radio/sx1276_rx.c`** (verified against every link line).
- `bench/host_proto/tx_len_guard.c`: `platform_now_ms` stubbed at `:32` (constant 1000), `host_cfg_profile_active` returns NULL at `:48` — so `sx1276_tx_begin` takes the BENCH branch (`tx.c:183-193`) and the new FHSS-clock refresh code compiles but never executes there. "Green with no stub edits" confirmed and strengthened.
- `bench/host_proto/fhss_scheduler_vectors.c`: exactly **14** `consider_remote` call sites at `:441-603` ("~14" → exact); epoch-wrap boundary test at `:570-591`.
- `bench/host_proto/rx_counters.c`: `test_dim_matches_enum` at `:209-218` pins `DIM == REJECTED_EPOCH_DRIFT + 1` and will fail by design on the enum append — confirmed.
- `fhss_stub.c` does not stub `consider_remote`; nothing anywhere stubs it (repo-wide grep: only `sx1276_rx.c`, `sx1276_fhss.c/h`, `sx1276_rx.h`, `fhss_scheduler_vectors.c`, `rx_counters.c` touch the symbol).
- `sx1276_rx_consider_remote_counts` has **zero** consumers under `host/` (grep: no matches) — no wire change. `base_station/lora_proto.py:625-636` is confirmed the legacy **8-channel** key_id scheme (`perm = list(range(8))` at `:626`) — unrelated to this scheduler. `include/lora_pkt_hdr.h:33-36` confirms MIC is schema_ver=2 future work.
- Doc grounding: F6 wording at `CONTROL_PLANE_DESIGN.md:414` (cites `sx1276_rx.c:214` — now drifted to `:217`; map's number is current). F1 = DTS slot clock / virtual grid at `:424` — map's Batch-2 interplay note is grounded.
- γ-1 claim: `sx1276_rx_tick` is gated on LOCKED (`rx.c:333`) and on clock-invalid (`:347-349`), and its DO path walks `sx1276_fhss_next_channel` (`:374-394`) — so a re-locked desynced node (LOCKED, clock invalid) does have γ-1 walking the stale local sequence. Accurate.

## Corrections

**C1 (substantive — the map's recovery loop does not close).** The map's §2 fix claims "Persistent disagreement now demotes in ≤2 s → BEGIN_SCAN resets the clock → UNANCHORED → next frame SNAPPED". This is defeated by the TX-side lazy anchor the map itself preserves: `sx1276_tx.c:207-214` re-anchors the clock **from the local scheduler** whenever `clock_valid()==0`. After the demotion resets the clock (`rx.c:640-644`), the node's very next TX re-validates it on the *same stale grid*; with the map's §3.5 TX-recency-refresh it then stays FRESH indefinitely while transmitting. Result: on any duplex node (this control plane is duplex — commands down, telemetry up), the UNANCHORED recovery tier is unreachable, every remote frame is `REJECTED_LOCKED_OUT`, and the demote/re-anchor cycle repeats forever — the fix converts today's Defect A into a different permanent desync. Same hole blocks cold-boot join for a node that transmits before its first receive (self-anchored → FRESH → refuses the established network's grid → SCANNING never locks → 30 s FAIL, `rx.c:519-562`). The corrected map adds a caller-side **grid-adoption flag** that forces the UNANCHORED tier until the first ALIGNED/SNAPPED after each acquisition reset/demotion/boot. The map's "both-fresh split-brain … backstop: demotes in 2 s, goes UNANCHORED, adopts" claim is wrong as written and correct only with this flag.

**C2.** "the six stub-linked cfg targets" — **five** targets link `fhss_stub.c` (Makefile `:353, :365, :588, :606, :653`: cfg-contract-unit, cfg-contract-wire, cfg-profile, cfg-profile-wire, cfg-clamp-fuzz), and only three of those are in the 25-target `check` list (the two cfg-contract targets are absent from `:246`). Conclusion unchanged: none can break.

**C3.** `fhss_stub.c` does not define "only" init/reset/scan_reset — it also defines `sx1276_airtime_set_budget_us` (`:83-85`) and four `sx1276_stub_*` capture accessors (`:41-55, :64-69, :87-89`). The load-bearing conclusion (consider_remote unstubbed) is confirmed.

**C4.** `rx_counters.c` `all[]` decision list is at `:61-67`, not `:71` (that line is the `record` call). The interleaved pattern at `:175-186` should also gain the new value.

**C5.** "`radio/sx1276_rx.c` (the one TU that includes both headers)" — false uniqueness: `sx1276_tx.c` also includes both (`sx1276_fhss_clock.h` at `tx.c:17`; `sx1276_rx_scan_policy.h` transitively via `sx1276_rx.h` at `tx.c:9`). Assert placement in `rx.c` remains fine.

**C6 (trivial).** Enum-stability warning wording is at Makefile `:698-701` (check-rx-retune-policy comment), not `:697-699`.

---

# CORRECTED IMPLEMENTATION MAP — F6: epoch-drift lock-out

All paths relative to `C:/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/` unless noted.

## 1. The failure today (two coupled defects, both verified in code)

**Defect A — permanent desync (the doc's F6 wording, `CONTROL_PLANE_DESIGN.md:414`).** `sx1276_rx_service()` calls `sx1276_fhss_consider_remote(parsed.epoch, parsed.hop_idx)` at `radio/sx1276_rx.c:174-176`, then **unconditionally** calls `scan_feed_frame(true)` at `radio/sx1276_rx.c:217` — even when the decision was `REJECTED_EPOCH_DRIFT` (gate at `radio/sx1276_fhss.c:232-236`, tolerance `SX1276_FHSS_SNAP_MAX_EPOCH_DRIFT 1U` at `include/sx1276_fhss.h:141`). Consequences chain:
- In LOCKED, `FRAME_VALID` → `REANCHOR` (`radio/sx1276_rx_scan_policy.c:122-127`), refreshing `channel_entry_ms`, so the `SX1276_RX_SCAN_LOCK_LOSS_MS` (2000 ms, `include/sx1276_rx_scan_policy.h:77`) demotion at `sx1276_rx_scan_policy.c:137-143` is starved whenever rejected frames keep arriving.
- Even when demotion **does** fire (coincident receives are typically >2 s apart on disjoint permutations — traffic-dependent, unmeasured), recovery is incomplete: the LOCKED→BEGIN_SCAN edge resets only the **clock** (`radio/sx1276_rx.c:640-644` → `sx1276_fhss_clock_reset()`), never the **scheduler epoch** (`s_fhss.epoch`, `radio/sx1276_fhss.c:45`; the BEGIN_SCAN dispatch at `rx.c:452-459` resets only the chantab walker). The next frame received during SCANNING is again `REJECTED_EPOCH_DRIFT`, yet `scan_feed_frame(true)` fires → `SCANNING+FRAME_VALID → LOCK` (`sx1276_rx_scan_policy.c:54-59`) — the node re-locks desynced with an invalid clock, and γ-1 (`sx1276_rx.c:325-404`: runs exactly when LOCKED ∧ clock-invalid, `:333,:347`) walks its own stale sequence via `next_channel` (`:374-394`). Nothing ever moves the local epoch toward the remote: **permanent, cycling desync**. The doc's "clock reset that would recover" is itself insufficient; recovery requires accept-when-unadopted (below).

**Defect B — healthy follower dragged (this item's concern).** Any well-formed header within ±1 epoch is `SNAPPED` (`radio/sx1276_fhss.c:242-244` → `snap_to`), and the caller then re-anchors the phase clock to the remote's grid (`radio/sx1276_rx.c:196-212`), regardless of local clock health. A peer with a skewed grid (or a forged v1 header — no MIC until schema_ver=2, `include/lora_pkt_hdr.h:33-36`) can walk a locally healthy clock one snap at a time.

**Defect A′ — the naive fix's own trap (found in verification).** Any "reject when local clock is fresh" gate interacts with the TX lazy anchor at `sx1276_tx.c:207-214`: whenever `clock_valid()==0`, the next TX re-anchors the clock **from the local scheduler**. So a clock reset by the demotion edge is re-validated — on the same stale grid — by the node's next transmission, and a freshness gate keyed on anchor recency alone then locks the node out of ever adopting the peer. Duplex nodes (all nodes here) would desync permanently; a cold-booted node that transmits before its first receive could never join (SCANNING never locks → 30 s FAIL path, `rx.c:519-562`). The design below closes this with an explicit adoption flag.

## 2. Design: three-tier snap gate keyed on local clock health, plus a grid-adoption flag

Health is computed by the **caller** (`sx1276_rx.c`) from existing state — the slot-clock anchor (`s_clk.valid`, `s_clk.anchor_ms`, `radio/sx1276_fhss_clock.c:8-14`) — plus one new file-static flag:

- **`s_grid_adopted`** (new, `sx1276_rx.c` file-static, bss=0): set to 1 when `consider_remote` returns ALIGNED or SNAPPED; cleared at boot (bss), on the LOCKED→BEGIN_SCAN demotion edge (`rx.c:640-644` block), and in `sx1276_rx_scan_reset()` (`rx.c:721-736`). While 0, the caller passes health = UNANCHORED **even if the clock is TX-self-anchored**. Semantics: "no remote grid accepted since the last acquisition reset — the local grid has no authority to refuse one." This is what makes the recovery tier reachable on duplex nodes and lets a cold-booted TX-first node join.

| Local health (as passed by caller) | ALIGNED (exact match) | Disagrees, Δepoch ≤ 1 | Δepoch > 1 |
|---|---|---|---|
| **UNANCHORED** (`!clock_valid() \|\| !s_grid_adopted`) | ALIGNED (no-op) | **SNAPPED** (accept) | **SNAPPED** (accept — recovery path, fixes Defect A) |
| **FRESH** (valid ∧ adopted ∧ age ≤ FRESH_MS) | ALIGNED (no-op; phase re-anchor as today) | **REJECTED_LOCKED_OUT** (new — fixes Defect B) | REJECTED_LOCKED_OUT |
| **STALE** (valid ∧ adopted ∧ age > FRESH_MS) | ALIGNED | SNAPPED (legacy ±1 rule) | REJECTED_EPOCH_DRIFT (legacy) |

- `FRESH_MS = 2000U` (`SX1276_FHSS_CLOCK_FRESH_MS`, new, in `include/sx1276_fhss_clock.h`), deliberately equal to `SX1276_RX_SCAN_LOCK_LOSS_MS`: a clock is authoritative exactly as long as the link-loss demotion would not yet have fired. Pin equality with a `_Static_assert` in `radio/sx1276_rx.c` (it includes both headers — note `sx1276_tx.c` does too, via `sx1276_rx.h`; rx.c is simply where the health computation lives).
- Age = `now_ms - anchor_ms` (u32 wrap-safe subtraction, same idiom as `sx1276_fhss_clock.c:35`). `anchor_ms` is a slot-*start* time, so age overstates by up to `toa + slot_offset` (~0.37 s worst at BW250, from `rx.c:204-207` anchor math) — irrelevant at 2000 ms; document it.
- Precedence in `consider_remote`: NOT_INIT > BAD_HOP > ALIGNED > health tiers (matches current ordering `fhss.c:212-227`), so lock-step peers still count ALIGNED and malformed hops are rejected even when unanchored.
- Coupled fix for Defect A: feed the scan SM `FRAME_VALID` **only** for ALIGNED/SNAPPED; all REJECTED_* feed `FRAME_INVALID` (LOCKED+INVALID → HOLD, `sx1276_rx_scan_policy.c:146-150`, so the demotion countdown runs; SCANNING+INVALID → HOLD, `:61-69`, so a rejected frame can no longer LOCK). Persistent disagreement now demotes in ≤2 s → BEGIN_SCAN resets clock **and clears `s_grid_adopted`** → next heard frame is UNANCHORED-tier → SNAPPED (any epoch) → anchor + adopted=1 + LOCK. The recovery loop closes **because the adoption flag survives TX re-anchoring** (Defect A′).
- Payload delivery unchanged: rejected headers still deliver the frame upward (existing rule, `radio/sx1276_rx.c:166-173`).
- TX-side anchor refresh (convergence direction): refresh the anchor's *recency* on every FHSS TX so a busier node that has adopted a grid reads FRESH and does not get dragged by an idler. Note the refresh only extends FRESH for nodes with `s_grid_adopted==1`; it cannot re-arm lock-out after a demotion.

## 3. Exact changes

1. **`include/sx1276_fhss.h`**
   - New enum `sx1276_fhss_clock_health_t { SX1276_FHSS_CLOCK_UNANCHORED=0, SX1276_FHSS_CLOCK_STALE=1, SX1276_FHSS_CLOCK_FRESH=2 }` — defined **here, not in the clock header**, so `sx1276_fhss.c` stays free of clock includes. This is load-bearing for the bench: `check-fhss-scheduler` (Makefile:413-421) links only `sx1276_fhss.c` + `sx1276_fhss_chantab.c`, no clock TU.
   - Append `SX1276_FHSS_SNAP_DEC_REJECTED_LOCKED_OUT = 5` to `sx1276_fhss_snap_decision_t` (`:118-134`). **Append only** — counters index by numeric value (stability-warning precedent: Makefile:698-701).
   - Change signature: `sx1276_fhss_consider_remote(uint32_t remote_epoch, uint8_t remote_hop_idx, sx1276_fhss_clock_health_t local_clock_health)`. Update the contract comment (`:143-165`) and the drift comment (`:136-141`).
   - Fix the stale comment at `:36-39`: the claimed mirror `bench/host_proto/fhss_scheduler.py` **does not exist** (verified; only `fhss_chantab.py` — golden vectors are C-only in `fhss_scheduler_vectors.c`).
2. **`radio/sx1276_fhss.c`** — `sx1276_fhss_consider_remote()` (`:210-245`): implement the table. UNANCHORED path bypasses the drift gate; FRESH path returns `REJECTED_LOCKED_OUT` before the drift computation; STALE path keeps `:229-244` verbatim. No state mutation on any REJECTED_*.
3. **`include/sx1276_fhss_clock.h` + `radio/sx1276_fhss_clock.c`**
   - New: `#define SX1276_FHSS_CLOCK_FRESH_MS 2000U`.
   - New: `uint32_t sx1276_fhss_clock_age_ms(uint32_t now_ms)` → `now_ms - s_clk.anchor_ms` (caller gates on `clock_valid()`).
   - Update the "Anchoring discipline" comment (`.h:34-42`): anchor recency now also refreshed by TX; adoption of a remote grid is health-gated and requires the RX-side adoption flag (supersedes the unconditional "whoever hears the other adopts the other's grid", `:39-40`).
4. **`radio/sx1276_rx.c`**
   - New file-static `uint8_t s_grid_adopted;` in the routed block (near `:53-59`).
   - In the consider_remote block (`:161-218`): compute health once — `(!sx1276_fhss_clock_valid() || !s_grid_adopted) ? UNANCHORED : (sx1276_fhss_clock_age_ms(platform_now_ms()) <= SX1276_FHSS_CLOCK_FRESH_MS ? FRESH : STALE)` — and pass it; on `dec == SNAPPED || dec == ALIGNED`, set `s_grid_adopted = 1U`.
   - Replace unconditional `scan_feed_frame(true)` at `:217` with `scan_feed_frame(dec == SX1276_FHSS_SNAP_DEC_SNAPPED || dec == SX1276_FHSS_SNAP_DEC_ALIGNED)`.
   - Clock anchor block (`:196-212`) unchanged — SNAPPED-from-UNANCHORED validates the clock, which is the recovery.
   - Demotion edge (`:640-644`): add `s_grid_adopted = 0U;` beside `sx1276_fhss_clock_reset()`. Same in `sx1276_rx_scan_reset()` (`:721-736`).
   - Add `_Static_assert(SX1276_FHSS_CLOCK_FRESH_MS == SX1276_RX_SCAN_LOCK_LOSS_MS, ...)`.
5. **`radio/sx1276_tx.c`** — in the FHSS branch of `sx1276_tx_begin()` after the abs_now snap (`:215-221`): add `sx1276_fhss_clock_anchor(tx_now_ms - sx1276_fhss_clock_in_slot_ms(tx_now_ms), abs_now);`. Phase-exact by the floor identity (verified: with `d = t - anchor = q*SLOT + r`, the re-anchor leaves `abs_slot()`/`in_slot_ms()` bit-identical for all later times, including the `:222-224` `slot_offset` read); only recency changes. No-op-equivalent on the first-TX path where `:207-214` just anchored at `tx_now_ms`.
6. **`include/sx1276_rx.h`** — bump `SX1276_RX_CONSIDER_REMOTE_COUNT_DIM` 5U→6U (`:48`); update the "(0..4)" comment (`:38`) and the REJECTED_* enumeration in `:44-47`. Counter TU `radio/sx1276_rx_counters.c` needs no code change (DIM-driven: `:23,:36,:42`).

No registers touched; no RX_FRAME_URC or any wire change (verified: `sx1276_rx_consider_remote_counts` has **zero** consumers in `host/` — the B1-SUMMARY URC is still future work).

## 4. What could break (and what provably cannot)

- **Bench link sets (`mingw32-make check`, 25 targets — full list Makefile:246):**
  - `check-fhss-scheduler` (Makefile:413-421) compiles the **real** `sx1276_fhss.c`; exactly **14** `consider_remote` call sites in `fhss_scheduler_vectors.c` (`:441-603`) must gain the third argument (use `STALE` to preserve every legacy expectation — NOT_INIT/BAD_HOP precedence is unaffected by health).
  - `check-tx-len-guard` (Makefile:619-634) compiles the real `sx1276_tx.c`, `sx1276_fhss.c`, `sx1276_fhss_clock.c` under `-DLIFETRAC_FHSS_TX_ROUTED`. `platform_now_ms` is already stubbed (`tx_len_guard.c:32`) and `host_cfg_profile_active` returns NULL (`:48`), so the new TX refresh compiles but executes only the BENCH branch (`tx.c:183-193`). Green with no stub edits.
  - `check-rx-counters` (Makefile:685-692): `test_dim_matches_enum` (`bench/host_proto/rx_counters.c:209-218`) pins DIM == `REJECTED_EPOCH_DRIFT+1` and **will fail by design** until updated to `REJECTED_LOCKED_OUT+1`; also extend the `all[]` decision list (`:61-67`) and optionally the interleaved pattern (`:175-186`).
  - `check-fhss-clock` (Makefile:814-821): additive API, nothing breaks; add cases (below).
  - **Stubs:** `consider_remote` is not stubbed anywhere (repo-wide grep), so the signature change cannot break the **five** `fhss_stub.c`-linked targets (Makefile:353, :365, :588, :606, :653 — of which only cfg-profile/-wire/-clamp-fuzz are in the `check` list; the two cfg-contract targets run outside it). `fhss_stub.c` defines `sx1276_fhss_init/reset`, `sx1276_rx_scan_reset`, `sx1276_airtime_set_budget_us`, and capture accessors — none affected. No new stub symbols needed: all RX-side changes live in `sx1276_rx.c`, which **no bench target compiles** (verified against every link line).
- **Static asserts:** existing ones (`sx1276_fhss.c:14-18`, `sx1276_rx.c:30-31`) unaffected; one new equality assert added (§3.4).
- **Wire parsers / python mirrors / daemons:** none. `base_station/lora_proto.py`'s `fhss_channel_index/_hz` (`:625-636`) is the legacy **8-channel** key_id scheme (`perm = list(range(8))`, `:626`), not this scheduler; no daemon parses snap counters; A6a header layout untouched.
- **Behavioral risks to document in the commit:**
  - Both-adopted-fresh split-brain: two nodes on disagreeing grids, both FRESH, lock each other out. Backstop (now actually reachable): whichever stops hearing accepted frames demotes in 2 s → `s_grid_adopted=0` → UNANCHORED tier → adopts the other. If both demote near-simultaneously they can swap grids once; convergence then relies on timing asymmetry — bounded oscillation, same chase dynamics as today's always-snap, flag for the FHSS bench.
  - Genuine local clock glitch while FRESH: correction refused → deaf → 2 s demotion → reacquire. Bounded (~2 s + acquisition) vs today's silent drag.
  - UNANCHORED/unadopted accept-any widens the cold-boot forge-capture window (today a far-epoch forgery is rejected even at boot), and the adoption flag makes every post-demotion node briefly capturable. A forger able to out-radiate the peer wins either way; real fix is the schema_ver=2 MIC (`lora_pkt_hdr.h:33-36`). State this in the header comment.
  - F1 (DTS slot clock, `CONTROL_PLANE_DESIGN.md:424`) interplay: keep the gate inside `consider_remote` (profile-agnostic) so Batch 2 inherits it unchanged.

## 5. Test additions (all inside existing targets; no new Makefile targets)

- **`check-fhss-scheduler`** (`fhss_scheduler_vectors.c`): FRESH+hop-mismatch-same-epoch → `REJECTED_LOCKED_OUT`, epoch/slot/seed unmutated; FRESH+Δ=1 → `REJECTED_LOCKED_OUT`; FRESH+exact match → `ALIGNED`; UNANCHORED+Δ=4900 → `SNAPPED`, epoch adopted, slot=hop+1, warmup cleared; UNANCHORED+bad hop → `REJECTED_BAD_HOP` (precedence); STALE tier re-pins all legacy cases including the epoch-wrap boundary test (`:570-591`); NOT_INIT precedence over health.
- **`check-rx-counters`** (`rx_counters.c`): DIM=6 invariant; LOCKED_OUT slot increments in isolation; interleaved + saturation on the new slot.
- **`check-fhss-clock`** (`fhss_clock_test.c`): `age_ms` = 0 at anchor instant; wrap-safe age across u32 tick wrap; `FRESH_MS == 2000` pin; TX-refresh identity property — after re-anchoring at `(t - in_slot_ms(t), abs_slot(t))`, `abs_slot`/`in_slot_ms` bit-identical across a sweep of future times (pins §3.5).
- **Coverage gap (explicit):** the `s_grid_adopted` flag and the health computation live in `sx1276_rx.c`, which no bench target compiles — they are testable only on the FHSS air bench (extend the run-16..22 series: force a demotion, confirm re-adoption within one scan acquisition while both nodes keep transmitting).

## 6. Flash/RAM estimate

- Flash: consider_remote tier logic ~60-100 B; clock age fn ~16-24 B; TX refresh ~24-40 B; RX health computation + adoption-flag set/clear + gated feed + assert ~56-80 B → **~160-280 B total** on top of 38326 B (headroom ~146 KB of the 180 KB APP region, Makefile:204). Cortex-M0+ Thumb estimate, not measured.
- RAM: +4 B bss (counter slot 5) + 1 B bss (`s_grid_adopted`). No stack change (health is a scalar; `sx1276_rx_frame_t` assert at `rx.c:30-31` unaffected).

## 7. Uncertainty markers

- The STALE middle tier (keep legacy ±1) is a design interpolation — the roadmap specifies only the lock-out and the recovery. Alternative: treat STALE as UNANCHORED (simpler, wider spoof surface). Decide at review.
- The adoption-flag semantics (clear on demotion/scan-reset/boot, set on first accept) is the minimal closure of Defect A′ found in verification; an alternative is tracking anchor *source* (SELF vs REMOTE) inside the clock TU — cleaner layering, but it inverts the §3.5 busier-node directionality for TX-only nodes. The flag version keeps the clock TU untouched except for `age_ms`.
- FRESH_MS=2000 equality with LOCK_LOSS_MS is a coherence argument, not a measurement; the FHSS bench should confirm no demote/re-lock oscillation under normal two-way traffic, and specifically the split-brain grid-swap dynamics noted in §4.
- Defect A's "demotion never runs" (doc wording) vs "demotion cycles without recovery" (this reading): both manifestations are grounded in the cited lines; which bites depends on coincident-receive rate, traffic-dependent and unmeasured.
- Flash figures are instruction-count estimates; verify with `make size` after landing.

---

# ITEM F7-phase-bias

VERIFICATION REPORT — F7-phase-bias implementation map

I checked every file:line claim against the tree. The map is substantially sound: all structural claims (sampling site, FIFO ordering, truncation site, link sets, python mirrors, pinned tests, geometry constants) verified against code. I recomputed all three ToA figures from `sx1276_airtime_compute_toa_us` (`sx1276_airtime.c:71-132`) and they are exact: 255 B SF7/BW500/CR4/5/pre8 = 99 904 µs (matches `CONTROL_PLANE_DESIGN.md:101`), 255 B BW250 = 199 808 µs, 215 B BW250 = 169 088 µs. The rounding vectors (99 499→99, 99 500→100, anchor 10000−100−13=9887) are arithmetically correct.

Corrections found (7), with evidence:

1. **RESOLVED UNCERTAINTY #1 — LBT is runtime-DISABLED in every daemon-driven run.** The colleague marked this unknown; the evidence exists. Both daemons force `CFG_KEY_LBT_ENABLE=0` at link open: `base_station/image_rx_daemon.py:423-426` (run-32 root cause comment: "firmware boots with LBT enabled… every command TX died ABORT_LBT/FORBIDDEN") and `firmware/tractor_x8/image_tx_daemon.py:588`; documented at `SETTINGS_REFERENCE.md:391, 517-518`. So the operative steady-state TX bias is the **~2.3–3.6 ms LBT-off figure**, not 9–14 ms. The 9–14 ms case is real but confined to the boot-to-CFG_SET window and any host that omits the disable.
2. **"LBT backoff pushes key-up across the slot boundary" is mechanically wrong.** LBT BUSY/BACKOFF *refuses* the TX at `sx1276_tx.c:285-292` — no key-up occurs. The in-call delay on the CLEAR path is CAD-until-clear + `platform_delay_ms(4)` RSSI dwell + mode transitions (`sx1276_lbt.c:102-139`); CAD timeout returns ERROR → refuse. Since worst success-path lag (~9–12 ms) is below the 15 ms guard, an actual straddle (offset > 200) is a defensive edge case, not an expected path. The stored-boundary pattern is still the right design (it also covers main-loop latency between the `host_cmd.c:492/:923` advisory and `tx_begin`, and future F2 mute-gate delays).
3. **Fail-closed exits misdescribed.** ":145-152 and :229-239 (keep `s_hop_slot_offset_ms = 0U` there)" — those exits zero hop_idx/epoch/freq/channel but **never touch `s_hop_slot_offset_ms`** today (only the BENCH/DTS branch does, `:193`). Harmless — they return before header pack — so `s_slot_boundary_valid = 0U` is *required* only in the BENCH/DTS branch; in the fail-closed exits it is defensive only.
4. **Line-number nits.** The stale "200 µs" text is at `sx1276_tx.c:38` (block 33-44; map said 37-44, ok) and `sx1276_rx.c:261-262` (map said 263-266). RX_FRAME_URC is `host_cmd.c:955-975` (correct; design doc says 976).
5. **Misattribution: "the 215 B fragment of doc §3."** Doc §3 (`:101`) uses 247 B body / 255 B on-air. 215 B is the FHSS slot-fit frame from the slot-clock work (`fhss_clock_test.c:107` "full 215 B frame (~173 ms ToA)", `sx1276_fhss_clock.h:23`). The 169 088 µs number itself is correct.
6. **"Snap outcomes cannot change" is overstated.** `sx1276_fhss_consider_remote` (`sx1276_fhss.c:210-245`) compares the remote header against **local scheduler state**, which the clock drives via `sx1276_rx_slot_follow`; a ≤14 ms anchor shift can flip ALIGNED↔SNAPPED for packets landing within ~14 ms of a boundary. The conclusion still holds because every consumer treats the two identically (`sx1276_rx.c:196-197` accepts both; only the per-decision counters differ) and the ±1-epoch drift gate (10 s) is unreachable by a 14 ms shift.
7. **Cosmetic:** `fhss_clock_test.c` currently contains **25** CHECKs while the string says "24 cases" (`:125`) — the count has already drifted once, confirming it is not asserted. Also "CAD ≈ (4+2)·t_sym": `cad_symbols` (default 4) sets the **RSSI dwell in ms** (`sx1276_lbt.c:89, :137`), not CAD length; HW CAD is ~2 t_sym. Ballpark total unchanged. Test-2 additionally needs `sx1276_fhss_init()` called (real scheduler linked; otherwise every TX refuses NOT_INIT at `:229-238`) — the map omitted this.

Everything else checked out exactly: `:206/:222-224/:259-264/:278-281/:284/:312/:327/:386-413/:397-409/:406/:416/:497-541`; `sx1276_modes.c:80-142` (5 ms retry deadline at `:108`); `config.h:30`; `sx1276_rx.c:198-207/:30/:174-176/:219-232/:690-719`; `sx1276_fhss_clock.h:52/:58/:68`; `lora_pkt_hdr.c:12` byte 3; Makefile `check` = exactly 25 targets (`:246`), `sx1276_rx.c` only at `:76` (zero bench links), fhss_clock linked at `:630` and `:814-818`; `tx_len_guard.c:31/:32/:48/:56/:68-72`; `fhss_stub.c:6-10` collision rule (plus `sx1276_airtime_set_budget_us`/`sx1276_rx_scan_reset` collisions at `:83/:95`); bench `lora_pkt_hdr.c:111-167` byte-3 pins incl. 199; `host_cmd.c:492/:923`; `lora_proto.py:835-836` and zero slot_offset consumers across `base_station/`; RegPaRamp (0x0A) never written; `sx1276.c:228-232` RS-3.6 2 MHz SPI; `main.c:148`; reserve() internally re-calls the 5-register estimate (`sx1276_airtime.c:203` + `:168-172`) = 10 reads total; the "1.1–1.4 ms Phase-A residual" decomposition is consistent.

---

CORRECTED IMPLEMENTATION MAP — F7 "one-sided phase bias" (design doc §10 Batch 1, `CONTROL_PLANE_DESIGN.md:415`)

All paths relative to `C:/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/` unless noted.

## 1. The failure, grounded in code as it exists today

**TX half — offset sampled at admission, not key-up.** `radio/sx1276_tx.c:206` samples `tx_now_ms = platform_now_ms()`; `:222-224` derives `s_hop_slot_offset_ms` from that same instant. Between that sample and actual RF key-up the code executes, in order:

| step | file:line | cost |
|---|---|---|
| standby/disarm (`sx1276_modes_apply`: RF-switch GPIO + 10 µs + 3 SPI writes + opmode readback retry loop, 5 ms deadline at `sx1276_modes.c:108`) | `sx1276_tx.c:259-264` → `sx1276_modes.c:80-142` | ~0.1 ms typ, up to 5 ms retry window |
| retune + PLL settle busy-wait | `sx1276_tx.c:278-281`, `SX1276_TX_PLL_SETTLE_US = 1000UL` at `:45` (comment at `:38` still says "200 µs" — stale) | 1.0 ms |
| LBT when enabled (build default `LORA_FW_LBT_ENABLE 1`, `config.h:30` — **but see §3: both daemons disable it at runtime**): one HW CAD (~2 t_sym) + RX-cont + `platform_delay_ms(cad_symbols)` = 4 ms RSSI dwell + standby, 2–4 extra mode transitions | `sx1276_tx.c:284` → `sx1276_lbt.c:82-149` (dwell at `:137`) | ~7–9 ms typ (BW250) on the CLEAR path; CAD phase bounded by the 20 ms timeout (`:17`), which aborts as ERROR → TX refused, not delayed |
| 2× ToA estimate = 10 SPI reg reads (`:312` direct; `:327` `sx1276_airtime_reserve` → internal estimate, `sx1276_airtime.c:203`, 5 reads each `:168-172`) | `:312`, `:327` | ~0.1–0.2 ms |
| FIFO load: 3 reg writes + 8 B hdr burst + ≤247 B payload burst at 2 MHz SPI (4 µs/B, RS-3.6 note `sx1276.c:228-232`) | `:386-413` | ~1.06 ms full fragment |
| `modes_to_tx` + FSTX PLL (~50 µs) + PA ramp (RegPaRamp 0x0A never written anywhere in the tree → POR 40 µs) | `:416` | ~0.2–0.3 ms |

So the header's `slot_offset_ms` (struct field committed at `:406`, packed to FIFO byte 3 per `lora_pkt_hdr.c:12`) understates the true key-up phase by **~2.3–3.6 ms with LBT off (the operative runtime state — §3), ~9–14 ms in any window where LBT is still on**. The structural cause: the phase byte is *sampled* before every delay in the table and *committed to the FIFO* at `:397-409`, before TX entry — nothing downstream can correct it.

**Important refusal note:** an LBT BUSY/BACKOFF outcome *refuses* the TX outright (`:285-292`) — LBT delays key-up only on its CLEAR path. This bounds the lag on frames that actually fly.

**RX half — µs→ms truncation.** `radio/sx1276_rx.c:199-201`: `toa_ms = sx1276_airtime_estimate_toa_us(rx_len) / 1000UL`, then `:204-207` anchors `slot_start = now - toa_ms - slot_offset_ms`. Truncation discards up to 0.999 ms. Verified against the firmware ToA model (`sx1276_airtime.c:71-132`): a 255 B on-air DTS fragment at SF7/BW500/CR4/5/preamble 8 = 5184 + 1280·74 = **99 904 µs** (matches doc line 101), so `/1000` loses exactly **0.904 ms**. On today's FHSS/BW250 grid: 255 B → 199 808 µs, loses 0.808 ms; the 215 B FHSS slot-fit frame (the "~173 ms" max-fit case, `fhss_clock_test.c:107` — *not* a doc-§3 figure; doc §3 uses 247 B body) → 169 088 µs, loses only 0.088 ms (payload-length dependent, 0–0.999 ms).

**Why both push the same way.** Receiver computes `slot_start_est = rx_done − toa − advertised_offset`. Real key-up = boundary + advertised_offset + tx_lag, so `slot_start_est = boundary + tx_lag + frac(toa)`. The follower grid (`sx1276_rx.c:690-719`) is therefore **late** by the sum; the receiver arms each new slot late, shrinking the 12 ms TX head-start (`SX1276_FHSS_SLOT_TX_HEADSTART_MS`, `sx1276_fhss_clock.h:68`) that exists precisely so the receiver is armed before the sender keys (run-20 silent-loss evidence in that comment block). In an LBT-on window the ~9–14 ms bias can consume nearly the *entire* 12 ms head-start.

A third lateness source — `now_anchor_ms` is sampled at main-loop service time (`sx1276_rx.c:198`, dispatched from `main.c:148`), not at the DIO0 edge — is real but unquantified here; see §6.

## 2. Exact changes

### 2a. TX: sample the phase byte at header-pack time, measured from the admission slot's boundary

File: `radio/sx1276_tx.c`, function `sx1276_tx_begin()`.

1. New file-statics (inside `#ifdef LIFETRAC_FHSS_TX_ROUTED`, next to `s_hop_slot_offset_ms` at `:66`):
   ```c
   static uint32_t s_slot_boundary_ms;        /* local time of the admission slot's boundary */
   static uint8_t  s_slot_boundary_valid;
   ```
2. In the FHSS branch (`:194-253`): after the snap at `:215-221`, replace the `:222-224` offset computation with recording the boundary:
   ```c
   s_slot_boundary_ms    = tx_now_ms - sx1276_fhss_clock_in_slot_ms(tx_now_ms);
   s_slot_boundary_valid = 1U;
   ```
   In the BENCH/DTS branch (`:187-193`): add `s_slot_boundary_valid = 0U;` next to the existing `s_hop_slot_offset_ms = 0U` (`:193`) — this is the **required** clear, since it is the only other path that reaches header pack. The fail-closed exits (`:145-152`, `:229-239`) return before the pack and do not touch `s_hop_slot_offset_ms` today; adding `s_slot_boundary_valid = 0U;` there is defensive only.
3. In the header-pack block (`:397-409`), immediately before `lora_pkt_hdr_pack`:
   ```c
   if (s_slot_boundary_valid != 0U) {
       const uint32_t off = platform_now_ms() - s_slot_boundary_ms;   /* wrap-safe u32 */
       s_hop_slot_offset_ms = (off > 255U) ? 255U : (uint8_t)off;
   } else {
       s_hop_slot_offset_ms = 0U;
   }
   ```
   **The trap to avoid:** do NOT re-call `sx1276_fhss_clock_in_slot_ms()` at pack time. If key-up ever slips past the slot boundary (defensive edge today — worst success-path lag ~9–12 ms sits under the 15 ms guard, but main-loop latency between the `host_cmd.c:492/:923` advisory and `tx_begin`, and the future F2 mute gate, both widen it), `in_slot_ms` wraps modulo 200 and the header would pair the *old* slot's (epoch, hop_idx) with an offset measured from the *next* boundary — corrupting the RX anchor by a full slot. Measuring against the stored admission boundary keeps offset and (epoch, hop_idx) consistent; a boundary-straddling frame honestly reports 200–235, which the u8 (≤255) represents and the linear RX anchor math handles — on old *and* new RX firmware alike.
4. Optional Phase B (residual model, +~1 ms accuracy on full fragments): add the still-unaccounted FIFO-burst + TX-entry lag: `off += ((uint32_t)effective_len * 4U + 300U + 500U) / 1000U;` (4 µs/B at 2 MHz SPI; ~300 µs modes_to_tx + FSTX + PA ramp). These constants are derived, not measured — gate Phase B on the F8 telemetry run and keep it a separately revertible commit.
5. Fix the stale "200 µs" comment text at `sx1276_tx.c:38` while touching the file (mirrored stale text at `sx1276_rx.c:261-262`).

### 2b. RX: round, don't truncate — via a new pure helper in the clock TU (bench-linkable)

File: `include/sx1276_fhss_clock.h` + `radio/sx1276_fhss_clock.c` — add:
```c
/* Anchor from a received A6a header: slot_start = rx_done_ms -
 * round(toa_us/1000) - slot_offset_ms. Pure; wrap-safe. */
void sx1276_fhss_clock_anchor_rx(uint32_t rx_done_ms, uint32_t toa_us,
                                 uint8_t slot_offset_ms, uint32_t abs_slot);
```
implemented as `sx1276_fhss_clock_anchor(rx_done_ms - ((toa_us + 500UL) / 1000UL) - (uint32_t)slot_offset_ms, abs_slot);` — calls only within its own TU, so no new external symbol anywhere.

File: `radio/sx1276_rx.c:198-207` — replace the inline `toa_ms` computation + `sx1276_fhss_clock_anchor(...)` call with:
```c
sx1276_fhss_clock_anchor_rx(now_anchor_ms,
                            sx1276_airtime_estimate_toa_us((uint8_t)rx_len),
                            parsed.slot_offset_ms, remote_abs);
```
(The followed-abs bookkeeping at `:210-211` stays as is.) Putting the math in the HW-free clock TU is what makes it pinnable: `sx1276_rx.c` is compiled by **zero** bench targets (it appears only in the firmware OBJS list, `Makefile:76`), while `sx1276_fhss_clock.c` is already linked by `check-fhss-clock` (`Makefile:814-818`) and `check-tx-len-guard` (`:630`).

No struct, register, or wire-layout changes. The 8 B header layout (`lora_pkt_hdr.h:10-24`, pack at `lora_pkt_hdr.c:5-18`) is untouched — only the *truthfulness* of byte 3 changes, which is exactly its documented meaning ("ms from the TX's slot boundary to TX key-up", `lora_pkt_hdr.h:17-23` — today's code violates that spec by recording admission time). Update that comment to note the offset may exceed 199 when key-up slips past the boundary.

## 3. Bias corrected (quantified)

**Runtime LBT state (resolves the old uncertainty #1):** the firmware *builds* with LBT on (`config.h:30`), but both daemons force `CFG_KEY_LBT_ENABLE=0` at link open — `base_station/image_rx_daemon.py:423-426` (run-32: with the tractor at ~58% duty every command TX died ABORT_LBT) and `firmware/tractor_x8/image_tx_daemon.py:588`; documented at `SETTINGS_REFERENCE.md:391/:517-518`. So steady-state runs are LBT-off; the LBT-on rows below apply only to the boot-to-CFG_SET window and any host that omits the disable.

| component | today (one-sided, late) | after Phase A | after Phase B |
|---|---|---|---|
| TX offset understatement, LBT off (operative steady state) | ~2.3–3.6 ms | ~1.1–1.4 ms (full frag), ~0.3 ms (skip frame) | ≲0.3 ms |
| TX offset understatement, LBT on (build default; boot window / non-daemon hosts only) | ~9–14 ms | same as above (LBT now inside the measured window) | ≲0.3 ms |
| RX ToA truncation | +0 to +0.999 ms (0.904 @ 255 B DTS; 0.808 @ 255 B BW250; 0.088 @ 215 B) | ±0.5 ms max, −0.096 ms on the 255 B DTS fragment | same |
| **total anchor lateness** | **~3–5 ms steady state (up to ~15 ms in LBT-on windows), always late** | **≲2 ms** | **≲0.8 ms, roughly symmetric** |

Against the margins: the steady-state bias eats ~25–40% of the 12 ms head-start (`sx1276_fhss_clock.h:68`), and an LBT-on window can consume it entirely; up to ~90% of the 15 ms guard (`:58`) in the worst case. Note F7 fixes *reporting and anchoring*; it does not stop key-up lag from eating the fit-check's assumption (`sx1276_tx_slot_wait_us`, `sx1276_tx.c:497-541`, consulted at `host_cmd.c:492` and `:923`) — optional hardening (subtract worst-case key-up lag in the fit check) is out of F7 scope.

## 4. What could break

- **Bench link sets (`mingw32-make check`, 25 targets — counted at `Makefile:246`):** the minimal fix references no new external symbol from `sx1276_tx.c` (`platform_now_ms` is already stubbed in `bench/host_proto/tx_len_guard.c:32`). `sx1276_fhss_clock_anchor_rx` is a new *definition* in an already-linked TU with no external calls — no unresolved refs anywhere. `check-tx-len-guard` runs the BENCH-profile path (`host_cfg_profile_active()` stub returns NULL → `tx_len_guard.c:48`), so the new FHSS-branch code is dormant there; its 248/247 admit/refuse pins (`:68-72`) are untouched.
- **`fhss_clock_test.c` pass-count string** (`"[PASS] fhss_clock: 24 cases"`, `:125`) should be updated when cases are added — cosmetic printf, not asserted; it is in fact *already* off by one (the file contains 25 CHECKs).
- **Pinned numbers:** `test_geometry` pins SLOT_MS=200, guard 15 ms, head-start 12 ms (`fhss_clock_test.c:99-110`) — F7 changes none. `bench/host_proto/lora_pkt_hdr.c:111-167` pins byte-3 placement/round-trip (incl. value 199) — layout unchanged, still green. Airtime vectors untouched. **No existing test pins the sampling instant or the truncation — nothing to un-pin; the current bug is test-invisible.**
- **Static asserts:** none apply (`sx1276_rx.c:30` frame-size assert and `include/static_asserts.c` memory-map asserts are untouched).
- **Wire parsers / python mirrors:** RX_FRAME_URC (`host_cmd.c:955-975`) and RFCO_PERTX are byte-identical. `base_station/lora_proto.py` mirrors only `LORA_HOP_HDR_LEN = 8` (`:835`; `:836` derives the 247 B ceiling); no python code parses `slot_offset_ms` (the firmware strips the header, `sx1276_rx.c:219-232`), and no daemon references it — confirmed by grep across `base_station/`. Nothing to mirror.
- **Behavioral:** the anchor shifts earlier by ≤ ~14 ms. `sx1276_fhss_consider_remote()` (`sx1276_fhss.c:210-245`) compares the remote header against *local* scheduler state, which the clock drives — so packets landing within ~14 ms of a boundary may flip ALIGNED↔SNAPPED. This is harmless: `sx1276_rx.c:196-197` treats both identically (anchor + follow + scan-feed), only the per-decision counters shift; and the ±1-epoch drift gate (10 s tolerance) is unreachable by a 14 ms move. Peers running old TX firmware against new RX firmware (and vice versa) interoperate — the anchor math is linear in the offset byte, so even a >199 straddle value decodes correctly on old RX; each half of the fix is independently compatible.
- **Sequencing with F8:** land F7 with or before F8, so the phase telemetry F8 exposes is truthful. F1 (DTS virtual grid) should reuse the boundary-timestamp pattern from §2a rather than reintroducing admission-time sampling.

## 5. Test additions

1. **`check-fhss-clock`** (`bench/host_proto/fhss_clock_test.c`, Makefile `:814-818`): new `test_anchor_rx()` — pin `99904 → 100` (the F7 number), `99499 → 99` / `99500 → 100` (half-up rule), `169088 → 169`, full anchor position (`rx_done=10000, toa=99904, off=13 → slot_start 9887`, `abs_slot` preserved, `in_slot_ms` phase checks), offset >199 (boundary-straddle, e.g. 215), and a u32-wrap-spanning `rx_done`. Fix the case-count string (see §4 — it is already stale).
2. **New target `check-tx-slot-offset`** (recommended; brings `check` to 26 targets): a `bench/host_proto/tx_slot_offset.c` modeled on `tx_len_guard.c` but with (a) a *mutable, advancing* fake clock — `tx_len_guard.c`'s constant `platform_now_us` (`:31`) would infinite-loop in `pll_settle_busy_wait` (`sx1276_tx.c:80-85`) once the FHSS branch retunes (`s_hop_freq_hz != 0`, `:278`), so the stub must advance per call; (b) `host_cfg_profile_active()` returning the FHSS profile id **and a direct call to the real `sx1276_fhss_init()` before the first TX** — otherwise `sx1276_fhss_next_channel()` returns NOT_INIT and every begin() refuses at `:229-238`; (c) an LBT stub that jumps the fake clock +8 ms before returning CLEAR (tx_len_guard's own stub pattern, `:42`); (d) a capturing `sx1276_write_burst` plus the real `radio/lora_pkt_hdr.c` (do not stub `lora_pkt_hdr_pack` as tx_len_guard does at `:56`); (e) a `host_uart_send_urc` stub (RFCO pertx emits under the FHSS profile, and the legal-dwell reserve path is live — real `sx1276_legal_dwell.c` is linked). Assert FIFO byte 3 reflects pack-time phase, not admission-time (with the advancing clock the expected value is deterministic; at minimum byte3 ≥ admission_offset + 8 + 1). Build with `-DLIFETRAC_FHSS_TX_ROUTED`; link the real `sx1276_tx.c / sx1276_fhss.c / sx1276_fhss_clock.c / sx1276_fhss_chantab.c / sx1276_airtime.c / sx1276_legal_dwell.c / host_rfco.c` + `sx1276_stub.c` (mirror of the `check-tx-len-guard` set, `Makefile:619-634`); must NOT link `fhss_stub.c` — symbol collision with the real `sx1276_fhss_init/reset` and `sx1276_airtime_set_budget_us` (rule documented at `fhss_stub.c:6-10`, `:71-75`, `:77-80`).
3. **Hardware verification** rides F8: once RX_FRAME_URC carries epoch/hop_idx/slot_offset, the tractor-side log directly measures residual anchor error; the RS-0.12 phase-sweep harness is the consumer. Because the daemons disable LBT at link open (§3), an LBT-on phase measurement requires deliberately skipping the daemon's CFG_SET — worth one run to bound the boot-window bias.

## 6. Flash/RAM estimate and marked uncertainties

- **Flash:** TX edit ~+50–100 B, clock helper ~+40–70 B, RX call-site ~neutral, optional Phase B ~+30 B, new bench target costs firmware nothing → **≈ +120–250 B** on top of 38 326 B (192K budget) — negligible. **RAM:** +5 B .bss (`s_slot_boundary_ms` + flag), no stack change (2.5K reserve unaffected).
- **Resolved (was uncertain):** runtime LBT state — both daemons force LBT off at link open (`image_rx_daemon.py:423-426`, `image_tx_daemon.py:588`), so daemon-driven bench runs see the ~2.3–3.6 ms TX bias; the ~9–14 ms figure applies only pre-CFG_SET or to non-daemon hosts.
- **Still uncertain:** (1) `sx1276_modes_apply` readback-retry latency on real HW (0–5 ms window, unmeasured); (2) Phase B constants (4 µs/B SPI, ~300 µs TX-entry) are derived from `sx1276.c:228-232` + datasheet defaults (RegPaRamp never written → assumed POR 40 µs), not bench-measured; (3) main-loop service latency in the RX anchor (`now_anchor_ms` at `sx1276_rx.c:198` is service-time, not DIO0-edge time) is an additional un-corrected lateness — fixing it means timestamping in the EXTI ISR and is deliberately out of F7 scope; (4) the doc's "−0.904 ms" is the DTS/BW500 255 B figure — on the *current* FHSS/BW250 grid the truncation is 0.808 ms (255 B) or 0.088 ms (215 B slot-fit frame), so the RX half of F7 matters most for the Batch-2 DTS schedule; (5) the LBT CLEAR-path timing breakdown (~7–9 ms) is an estimate — HW CAD duration and mode-transition costs are unmeasured; only its components' bounds (20 ms CAD timeout, 4 ms dwell) are code-pinned.

---

# ITEM F8-rx-frame-urc-telemetry

VERIFICATION RESULT: the map is substantially sound — every load-bearing file:line claim checked out against code read this session — with 3 substantive corrections, 2 additions, and several sharpenings.

## Corrections (with evidence)

1. **WRONG: the H7 "malformed" vector inverts meaning under the relaxed check.** `bench/h7_host_proto/mh_runtime_health_vectors.c:204` is `bad_rx_payload[8] = {3U, ...}` — len byte 3, payload_len 8, so 8 < 8+3=11. That is a *truncation* case, not "payload_len too large" as the map presumed. Under the relaxed `payload_len < 8+rx_len` check it is **still rejected** — the vector passes unchanged, nothing inverts. The work is additive (new vectors), not a deliberate rewrite of the existing one.
2. **UNPROVEN AS WRITTEN: fixture lines "prove all three regexes still match".** `test_parser_token_contract.py` checks only its own `CONTRACT` token-presence patterns (`__RX_FRAME__\b` etc., :28-38) against the fixture — it never imports or runs `analyze_rtt._RX_FRAME_RE`, `analyse_paired_sweep.rx_re`, or `w2_02_host_pipeline._RX_FRAME_RE`. Adding fixture lines alone proves nothing about them; the test must be extended to execute those three regexes. Also, adding `__RX_FRAME__` lines changes the pinned counts (RX_FRAME_v2=3, combined=5) — CONTRACT must be bumped in the same edit (the file header at :40-42 says exactly this).
3. **WRONG PATH + SCOPE: check_mh_wire_sync.py.** It lives at `DESIGN-CONTROLLER/tools/check_mh_wire_sync.py` (not under tractor_h7), and it compares `host_types.h` against **only the `murata_host/` copy** of mh_wire.h (`MH_WIRE` at :13) — the `src/` copy is invisible to it. Also, names added to `SYNC_KEYS` must exist in *both* files or CI fails immediately, so the SYNC_KEYS extension must land in the same commit as both defines.
4. **MISSED MIRROR:** `DESIGN-CONTROLLER/tools/mh_wire_constants.py` — auto-generated from mh_wire.h by `tools/gen_mh_wire_py.py`, checked in, **not** regenerated by CI (no hits in arduino-ci.yml). Regenerate after editing mh_wire.h. No functional risk: `tools/murata_host_l072_mock.py` never exercises RX_FRAME_URC (grep clean).
5. **MISSED SAFE-CLAIM (now verified):** H7 `murata_host_frame_t.payload[HOST_MAX_PAYLOAD_LEN=320]` (`murata_host.h:20`, `mh_wire.h:68`) — the 271-byte extended URC fits; no H7 buffer change needed. Also verified empirically: a host TU including `sx1276_rx.h` compiles clean under `gcc -Wall -Wextra -Werror` both with and without `-DLIFETRAC_FHSS_TX_ROUTED=1`, and `sizeof(sx1276_rx_frame_t)` is exactly 280 — the :30 static assert is tight but untouched by F8.
6. **Sharpenings:** map's uncertainty on the Method-G gate is resolved — CI builds the vectors with `-DLIFETRAC_USE_METHOD_G_HOST=1` unconditionally (arduino-ci.yml:185), so the vectors gate regardless of what the flashed H7 enables. `image_tx_daemon.py` imports are at :123-124 (block :88-129). `analyse_paired_sweep.py` is at repo-root `LifeTrac-v25/tools/`. New Makefile target must also join `.PHONY` (:146). Baseline flash: the checked-in `build/firmware.bin` is 22,816 B (Jul 29) vs the stated 38,326 B — build dir is stale relative to the flashed image; re-measure at land time (budget conclusion unaffected).

Everything else verified exact: sx1276_rx.c:30/:135/:148-160/:219-232; lora_pkt_hdr.h:17-23/:49-56; host_cmd.c:955-975; main.c:148-149; Makefile:60/:112/:246 (25 targets, none compiling host_cmd.c/host_uart.c/sx1276_rx.c); host_types.h:48/:80-92/:119; host_uart.h:12 (=311); config.h:48-49; host_uart.c:843-881 guard :852; sx1276_tx.c:187-193; mh_runtime_health.c:105 strict equality, both copies byte-identical; mh_runtime.c:105-112; SYNC_KEYS :17-63; parse_rx_frame v2 :1198-1218 / v1 :599-618; image_rx_daemon.py:88-97; fhss_rx_probe.py:15/:59; analyze_rtt.py:94-98; analyse_paired_sweep.py:31-34; w2_02_host_pipeline.py:292-293; fixture :49-53; lora_proto.py:835 with no RX_FRAME parsing; `__RX_FRAME__` emitters put payload_hex last (v2 :1519-1522, :2100-2103).

---

# F8 IMPLEMENTATION MAP — phase telemetry in RX_FRAME_URC (CORRECTED)

All claims grounded in code read this session; uncertainties flagged inline. Paths are relative to `C:/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER/` unless absolute.

## 1. The failure and the code path that exhibits it today

- `firmware/murata_l072/radio/sx1276_rx.c:148-160` — routed builds (`-DLIFETRAC_FHSS_TX_ROUTED=1`, unconditional in the ARM app CFLAGS, `Makefile:112`) parse the 8-byte hop header into `out_frame->hdr` (`lora_pkt_hdr_t`: `profile_id`, `hop_idx`, `epoch`, `slot_offset_ms` — `include/lora_pkt_hdr.h:49-56`).
- `radio/sx1276_rx.c:219-232` — the header is then stripped: payload shifted down by `LORA_PKT_HDR_LEN` (8), `out_frame->length` reduced. After this point epoch/hop_idx/slot_offset exist only inside the L072. (The design doc's F8 row cites `sx1276_rx.c:389-402` — stale; the strip is at 219-232 today.)
- `host/host_cmd.c:955-975` (`host_cmd_emit_rx_frame`) — serializes only `{u8 len, i8 snr, i16le rssi, u32le timestamp_us, payload[len]}` into a local `payload[264]` and sends URC 0x91 with length `8+len`. `frame->hdr` / `frame->hdr_valid` are never serialized. Sole call site: `main.c:148-149`.
- Result: slot alignment is unverifiable from either X8; the RS-0.12 bench plan explicitly works around it with tractor-side logging (design doc §10, sequencing note at CONTROL_PLANE_DESIGN.md:460-465).

Notes: in routed builds every *delivered* frame has `hdr_valid==true` (parse failure drops the frame, `sx1276_rx.c:152-158`; CRC error drops earlier, :102-119). Under DTS/bench profiles TX zeroes the hop fields before prepending the header (`sx1276_tx.c:187-193`), so F8 telemetry is real under FHSS and all-zeros under DTS — still useful once F1 (virtual DTS grid) lands. In the unrouted build `hdr` is never written (`sx1276_rx.c:135` sets only `hdr_valid=false`) — reading it would be uninitialized stack; the pack helper must gate on `hdr_valid`.

## 2. Exact appended fields and encoding (additive tail, +8 bytes)

Append after `payload[len]`, at URC-payload offset `8+len`:

| off | size | field | encoding |
|---|---|---|---|
| +0 | 1 | `phase_flags` | bit0 = hop header valid (`frame->hdr_valid`); bits 1-7 reserved, write 0 |
| +1 | 1 | `profile_id` | `hdr.profile_id`, 0 if invalid |
| +2 | 1 | `hop_idx` | `hdr.hop_idx`, 0 if invalid |
| +3 | 1 | `slot_offset_ms` | `hdr.slot_offset_ms` (u8, saturated at 255 by TX per `lora_pkt_hdr.h:17-23` wire comment) , 0 if invalid |
| +4 | 4 | `epoch` | u32 LE (`put_u32_le`, already used in host_cmd.c:965), 0 if invalid |

Rules:
- `payload[0]` (len) stays the radio-payload length, **excluding** the tail — every existing parser slices by it, which is what makes this additive.
- Tail always present (deterministic wire), zero-filled with flags=0 when `hdr_valid==false` (guards the unrouted-build garbage-hdr case above).
- Parsers detect presence by `total_len - (8+len) >= 8`. Future extensions append after these 8 bytes (STATS_URC additive discipline, `host_types.h:119`).

Budget check: max radio len 255 (RegRxNbBytes is u8; routed delivered max is 247 after strip) → max URC payload 8+255+8 = 271 ≤ `HOST_PAYLOAD_MAX_LEN` = 311 (`include/host_uart.h:12` = 320−7−2; `HOST_INNER_MAX_LEN` 320 at `config.h:48`); inner frame 271+7+2 = 280 ≤ COBS budget (`HOST_COBS_MAX_LEN` 325, `config.h:49`). No framing change to `host_uart.c` (`host_uart_send_urc` :843-881, guard :852 passes). H7 side: `murata_host_frame_t.payload[320]` (`murata_host.h:20`) — fits, no buffer change. UART cost: +8 B/frame ≈ +87 µs at 921600 — negligible.

## 3. Files + functions to change

**L072 firmware (`firmware/murata_l072/`):**
1. `host/host_cmd.c` `host_cmd_emit_rx_frame` (:955-975): grow local buffer `payload[264]` → `payload[272]`; delegate byte layout to a new pure pack helper; send `8 + len + 8`.
2. NEW `host/host_rx_wire.c` + `include/host_rx_wire.h`: `uint16_t host_rx_frame_urc_pack(const sx1276_rx_frame_t *f, uint8_t *out, uint16_t out_cap)` — pure, no HW/UART calls, mirroring the host_rfco "do NOT pack inline" precedent (`host_types.h:80-92`) and the `check-cfg-wire-owner` ownership discipline. Includes only `sx1276_rx.h` — **empirically verified host-safe**: a probe TU including it compiles clean under host gcc `-Wall -Wextra -Werror` with and without `-DLIFETRAC_FHSS_TX_ROUTED=1`. Alternative (smaller diff, weaker testability): pack inline in host_cmd.c and rely on H7-side vectors only.
3. `include/host_types.h:48`: extend the RX_FRAME_URC layout comment; add `#define HOST_RX_FRAME_URC_TAIL_LEN 8U` and `#define HOST_RX_FRAME_URC_PHASE_VALID 0x01U`.
4. `Makefile`: new `check-rx-frame-urc` target (26th) — add to the `check` aggregate (:246) AND to `.PHONY` (:146). Link set: `bench/host_proto/rx_frame_urc_vectors.c` + `host/host_rx_wire.c` only — **no stub impact** (helper is pure; verified from the full Makefile read: none of the 25 aggregate recipes compiles `host_cmd.c`, `host_uart.c`, or `sx1276_rx.c`; `host_cmd.c` appears only in the ARM SRCS list at :60).
5. NO change to `sx1276_rx.c` or `sx1276_rx_frame_t` — `hdr`/`hdr_valid` already exist (`sx1276_rx.h:28-29`); `_Static_assert(sizeof(...) <= 280)` at `sx1276_rx.c:30` untouched (sizeof is exactly 280 — measured; do not add fields to the struct).

**Tractor H7 (the one parser that BREAKS — see §4):**
6. `firmware/tractor_h7/murata_host/mh_runtime_health.c:105` AND the byte-identical copy `firmware/tractor_h7/src/murata_host/mh_runtime_health.c:105` (diffed this session: identical): change `if (frame->payload_len != (uint16_t)(8U + rx_len)) return false;` to `<`; parse the tail when `payload_len >= 8+rx_len+8` into new `mh_runtime_health_t` fields (`last_rx_phase_flags`, `last_rx_profile_id`, `last_rx_hop_idx`, `last_rx_slot_offset_ms`, `last_rx_epoch` — struct in `mh_runtime_health.h`, RX block at :48-52); default flags=0 for legacy-length frames. Optionally extend the `mh_runtime.c:105-112` `MH RX_FRAME` log line (both copies).
7. `firmware/tractor_h7/murata_host/mh_wire.h` (+ `src/` copy): mirror the two new defines. Sync tooling: `DESIGN-CONTROLLER/tools/check_mh_wire_sync.py` (NOT under tractor_h7) compares only `SYNC_KEYS` (:17-63) between `host_types.h` and the **murata_host copy only** — the src copy is unchecked by tooling, keep it in sync manually. Add the new names to `SYNC_KEYS` *in the same commit as both defines* (a partial landing fails CI: the checker requires each SYNC_KEY in both files). Also regenerate `DESIGN-CONTROLLER/tools/mh_wire_constants.py` via `tools/gen_mh_wire_py.py` (checked-in generated file, not CI-regenerated; no parser impact — the loopback mock never uses RX_FRAME).

**Python mirrors:**
8. `firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py:1198-1218` `parse_rx_frame` — the canonical copy imported by BOTH daemons (`base_station/image_rx_daemon.py:88-97`; `firmware/tractor_x8/image_tx_daemon.py:123-124`). Already tolerant (raises only when `len(payload) < 8+rx_len`, :1208). Extend: `tail = payload[8+rx_len:]`; if `len(tail) >= 8` add keys `phase_valid` (flags bit0), `profile_id`, `hop_idx`, `slot_offset_ms`, `epoch` (`<I` LE); else set them `None`.
9. `firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py:599-618` — legacy v1 duplicate, same tolerant shape; same optional extension (probes only).
10. `firmware/x8_lora_bootloader_helper/fhss_rx_probe.py:59` — prints raw URC payload length (will read +8); optionally print the phase fields. No change required for correctness.
11. Probe log lines: if `__RX_FRAME__` lines gain fields (e.g. `hop= epoch= slotoff=`), append them **after** `payload_hex=` — `analyze_rtt.py:94-98` and repo-root `LifeTrac-v25/tools/analyse_paired_sweep.py:31-34` pin the field order up through `payload_hex=` (both unanchored at end and hex-terminated captures, so trailing tokens are safe); `w2_02_host_pipeline.py:292-293` is tolerant (`.*?payload_hex=`). Emitters to touch: `method_h_stage2_tx_probe_v2.py:1519-1522` and :2100-2103 (payload_hex is last today).
12. `base_station/lora_proto.py` — **no change**: it never parses RX_FRAME_URC (grep verified); it only mirrors `LORA_HOP_HDR_LEN = 8` (:835) for TX body sizing (:836, :850), which is unaffected.

## 4. What could break

- **CONFIRMED breaker: the H7 strict-equality length check** (`mh_runtime_health.c:105`, both copies). The "host parsers ignore extra bytes" convention is FALSE for this one parser — it rejects any extended URC, so `rx_frame_count` freezes and runtime health sees a dead RX. Ordering: land+flash the relaxed H7 parser (tolerant of both lengths) BEFORE flashing the F8 L072 firmware on any bench with the Method-G runtime enabled. The health TU is compiled only under `LIFETRAC_USE_METHOD_G_HOST=1` (`mh_runtime_health.c:3`); whether the currently-flashed H7 build enables it is unverified from here, but CI builds the vectors with `-DLIFETRAC_USE_METHOD_G_HOST=1` unconditionally (`.github/workflows/arduino-ci.yml:185`, job `h7-host-driver-tests`, linking the `murata_host/` copy), so the vectors gate regardless.
- H7 CI vectors: the happy-path exact-length frame (`mh_runtime_health_vectors.c:84-105`) still passes under `<`. The existing "malformed" vector (:204-212, `bad_rx_payload[8]` with len byte 3) is a **truncation** case (8 < 11) and **survives the relaxation unchanged** — it does not invert. Required work is additive vectors (§5), not a rewrite.
- Bench link sets (murata_l072): none — no existing bench target compiles `host_cmd.c`, `host_uart.c`, or `sx1276_rx.c` (all 25 aggregate recipes read; they compile pure radio/host TUs plus `sx1276_stub.c`/`fhss_stub.c`). The new pure TU adds no undefined symbols.
- Static asserts: none touched (`sx1276_rx.c:30` unchanged; `include/static_asserts.c` covers the memory map, not URC layout).
- Wire parsers verified tolerant: `parse_rx_frame` v1/v2 (Python), `mh_runtime.c` (logs from the health struct only), the three `__RX_FRAME__` log regexes (trailing-token safe). `check_mh_wire_sync.py` unaffected unless SYNC_KEYS is extended (recommended, same-commit rule above).
- Deployment skew: the daemons import the helper from `/work` on the boards (`fhss_rx_probe.py:15` and equivalents) and the deployed tree is known stale (memory note) — re-push `method_h_stage2_tx_probe_v2.py` to both X8s or the daemons keep the old dict shape (harmless — tail ignored — but the telemetry stays invisible, defeating F8).
- Stack/RAM: +8 B transient stack in `host_cmd_emit_rx_frame` (264→272 local) on top of the existing 280 B `rx_frame` (measured) + 320 B `inner` in `host_uart_send_urc`; within the 2.5K reserve, no .bss growth.

## 5. Test additions

- **New murata_l072 bench target `check-rx-frame-urc`** (pattern of `check-rfco-pertx`, pure-TU variant): golden vectors pinning (a) first `8+len` bytes byte-identical to the legacy layout; (b) `payload[0]` excludes the tail; (c) `hdr_valid=false` → tail all-zero, flags=0 (pins the garbage-hdr guard); (d) `hdr_valid=true` → flags=0x01, epoch LE byte order, slot_offset=255 passthrough; (e) len=255 → return 271, no overflow at `out_cap` 272; (f) len=0 → 16-byte payload.
- **H7 `mh_runtime_health_vectors.c`** (new cases; keep the existing :204 truncation-reject as-is): extended frame (8+len+8) accepted, count increments, phase fields parsed; legacy frame (8+len) accepted with flags defaulted 0; truncated frame (`payload_len < 8+len`) still rejected; partial tail (8+len+3) accepted-as-legacy.
- **Python — `test_parser_token_contract.py`**: extending the fixture (:49-53) alone is NOT sufficient — the test runs only its own CONTRACT token counts, never the analyzer regexes. Do both: (i) add new-style `__RX_FRAME__ ... payload_hex=HEX hop=N epoch=N slotoff=N` fixture lines AND bump the pinned counts in CONTRACT (:28-38) in the same edit (RX_FRAME_v2 and combined counts are exact); (ii) add assertions that execute the three analyzer regexes (`analyze_rtt._RX_FRAME_RE`, `analyse_paired_sweep` rx_re, `w2_02_host_pipeline._RX_FRAME_RE` — import or duplicate) against a new-style line and confirm they match with payload_hex captured intact. Plus a direct `parse_rx_frame` case for tail present/absent (no existing unit file for it; smallest home is a new case in the contract test).
- **Bench-on-air acceptance** (post-flash): FHSS two-board run — every RX_FRAME_URC carries flags=1 with epoch/hop advancing and slot_offset ≤ guard; DTS run — flags=1, hop fields all-zero.

## 6. Flash cost estimate

Pack helper + tail writes + call overhead at `-Os` on Cortex-M0+: roughly **80–200 bytes** (byte stores + one u32 LE store + branch; no new data/bss). Baseline caveat: the stated 38,326 B app figure does not match the stale checked-in `build/firmware.bin` (22,816 B, Jul 29) — re-measure with `arm-none-eabi-size` after `make all` at land time. Either way the APP region is 180 KB and `tools/check_size_budget.py` (run by the `firmware-l072-cross-compile` CI job, ARDUINO_CI.md:35) is unaffected. Estimate is judgment, not measured.

---

# ITEM F9-opmode-sync-firstclass

Verification complete. Every file:line claim in the map was checked against the actual code this session. The map is substantially sound — the core failure chain, the Option B design, the link-set analysis, and the Makefile/stub claims all verified exactly. I found 2 factual corrections, 1 resolved uncertainty, and 3 sharpenings.

## Corrections found (with evidence)

**C1 — The bench-diag opmode value inventory is incomplete (§2, §6).** The map lists only 0x81/0x83 as bench-diag opmode writes. The method_h v2 **T2 TCXO test also writes 0x00 (FSK SLEEP) at `method_h_stage2_tx_probe_v2.py:1047`, 0x01 (FSK STDBY) at `:1059`, and restore 0x80 at `:1071`** (same in the older `method_h_stage2_tx_probe.py:449/:461/:473`). Consequence unchanged (probes need diag=1 builds), but the §6 diag=false test list must pin (0x01,0x00) and (0x01,0x01) refused, since real probes send those values.

**C2 — "Daemons treat the failure as warn-and-continue" is wrong for one call site (§1 point 3).** `image_rx_daemon.py:876-883` (the profile-switch re-arm) sits inside `_apply_profile`'s try block (`:861-888`); a refused write propagates to `:886-888` and makes `_apply_profile` return False, which the two-phase switch machinery treats as apply-failure and **reverts** (`:695-701`, revert watchdog `:515-523`). So flag=0 doesn't just deafen RX — it also poisons the profile-switch state machine even when the CFG activation itself succeeded. This strengthens the F9 case and belongs in §1.

**U1 — §8 uncertainty resolved.** `HostLink.request()` **raises RuntimeError immediately** on a seq-matched ERR_PROTO — `method_g_stage1_probe.py:327-330` (`raise RuntimeError(f"ERR_PROTO for req ...")`), not a timeout. This is what makes C2 bite fast, and it defines the Option-A fallback shape if that route were chosen.

**S1 — mh_wire.h exists in two copies.** `firmware/tractor_h7/murata_host/mh_wire.h:22` AND `firmware/tractor_h7/src/murata_host/mh_wire.h:22` both define `HOST_TYPE_REG_WRITE_REQ`; no H7 .c/.cpp/.ino references it (grep verified). Option A's churn cost is two headers, not one.

**S2 — the CFLAGS override claim in §4 is imprecise.** `CFLAGS :=` at `Makefile:106` is a simple assignment, so a command-line `make CFLAGS=-D…` would *replace* the whole flag set (losing MCU flags); the firmware-build override vector is `EXTRA_CFLAGS` (`Makefile:35`, used at `:186`). The `#ifndef` wrap is still required (a `-D…=0` against the bare `#define … 1` is a macro redefinition, fatal under `-Werror`), but the documented override should be `EXTRA_CFLAGS`. Also §4 omits that the new target must be added to `.PHONY` (`Makefile:146`) and needs a `*_BIN` variable alongside `Makefile:220-244`.

**S3 — the daemon "read-back+log" mitigation is overstated (§3).** Only the rx-daemon connect autowake (`image_rx_daemon.py:446`) and the probe autowake (`method_h...v2.py:1473`) read back post-write; both `_ensure_rxcont` implementations write blind (`image_rx_daemon.py:550-557`, `image_tx_daemon.py:729-736`). The actual mitigation is that `_ensure_rxcont` re-reads opmode on every idle/post-TX cycle, so an ignored write is re-attempted continuously — same conclusion, different mechanism.

Everything else checked out exactly: `host_cmd.c:519/:521-538/:567/:577-584/:586/:594-596/:817`; `config.h:60-61/:65-67`; `sx1276_modes.c:107-127/:125/:148-179`; `sx1276_rx.c:63-75`; `sx1276_tx.c:113-117/:124-125/:259-264`; `host_cfg_profile.c:184-192`; `main.c:64-75`; `Makefile:146/:246 (25 targets)/:261`; no `$(HOST_CC)` recipe compiles host_cmd.c or sx1276_modes.c; `tx_len_guard.c:53` mismatched-signature `sx1276_rx_arm` stub; `sx1276_stub.c:23-28`/`fhss_stub.c:1-11` collision rules; `static_asserts.c` (27 lines, no radio/host symbols); `host_types.h:17/:37/:220-252`; `SETTINGS_REFERENCE.md:316/:420/:428`; `CONTROL_PLANE_DESIGN.md:418` (§10 F9 row); TODO.md RS-4.12 note at ~:1064-1071; both daemons activate a profile on every connect (`image_rx_daemon.py:415`, `image_tx_daemon.py:579` → `configure_regulatory_profile_if_needed`, defined `method_h...v2.py:439`); `radio_sleep.py` writes 0x80; `w2_02_radio_wake_rxcont.py` writes 0x85; `base_station/lora_proto.py` has no REG_WRITE mirror; `tools/check_mh_wire_sync.py:28,38` and `tools/mh_wire_constants.py:41-42` carry the REG_WRITE keys.

---

# F9 IMPLEMENTATION MAP — make opmode-sync first-class (CORRECTED)

All claims verified by reading the cited files this session.

## 1. The failure this fixes, with today's code path

RS-4.12 has two halves: (a) daemons arm RXCONT by raw register write, (b) firmware folds that write into its tracked mode state so the TX path's re-arm snapshot sees it. Both halves live behind one diagnostic compile flag.

Working path today (`HOST_ALLOW_REG_WRITE_DIAG=1`, `config.h:61`):
- Python sends `REG_WRITE_REQ` (0x31) payload `[0x01, 0x85]` → dispatch `host_cmd.c:817` → `handle_reg_write()` `host_cmd.c:567` → `reg_write_allowed()` `host_cmd.c:519` (0x01 is in the 13-register diag allowlist, `:522`) → `sx1276_write_reg(0x01, 0x85)` `:586` → **sync hook** `host_cmd.c:594-596` → `sx1276_modes_sync_external(0x85)` (`sx1276_modes.c:148`) sets `s_state = SX1276_STATE_RX_CONT`.
- Next firmware TX: `sx1276_tx_begin()` snapshots state (`sx1276_tx.c:125`), sets `s_rearm_rx` (`:259`), and `sx1276_tx_cleanup()` re-arms RX after TX_DONE (`sx1276_tx.c:113-117`).

Broken path if the flag is set to 0 (which its own comment `config.h:60` "keep these conservative in production" instructs, and which `SETTINGS_REFERENCE.md:316` justifies — the same flag exposes RegModemConfig 0x1D/0x1E behind the FCC airtime invariant):
1. `reg_write_allowed()` compiles to `return false` for **every** register (`host_cmd.c:539-542`).
2. `handle_reg_write()` answers `ERR_PROTO FORBIDDEN` (`host_cmd.c:577-584`); the sync hook at `:594` becomes unreachable. `HostLink.request()` surfaces that as an immediate RuntimeError (`method_g_stage1_probe.py:327-330`) — fail-fast, not a timeout.
3. Most daemon call sites are warn-and-continue: `image_rx_daemon.py:451-452` (connect autowake), `:556-557` (`_ensure_rxcont`), `image_tx_daemon.py:612-614` (connect), `:735-736` (`_ensure_rxcont`) — the modem is never armed. **Exception:** the profile-switch re-arm `image_rx_daemon.py:876-883` sits inside `_apply_profile`'s try (`:861-888`); the raise makes the apply return False, and the two-phase switch machinery **reverts a profile switch that actually succeeded on the wire** (`:695-701`, watchdog `:515-523`) — flag=0 breaks profile switching too, not just RX arming.
4. The kill shot: every daemon connect drives a profile activation (`image_rx_daemon.py:415`, `image_tx_daemon.py:579` → `configure_regulatory_profile_if_needed`, `method_h_stage2_tx_probe_v2.py:439` — stages AND activates), and `host_cfg_profile_activate()` parks the modem in STANDBY (`host_cfg_profile.c:192`) and never re-arms — with the host write refused, **nothing can ever arm RX again**. Boot-time arming (`main.c:74`) does not save you past the first profile activation; a `LIFETRAC_BENCH_RADIO_IDLE_SLEEP=1` build (`main.c:69`) is deaf from boot. This is exactly the run-31 signature (rx_frames=1/300 s, `_ensure_rxcont` docstring `image_rx_daemon.py:541-548`).

So today production is forced to ship with the whole 13-register diag surface open just to keep RX arming (and profile switching) alive. F9 = decouple the production-critical opmode write from the diagnostic register surface.

## 2. What the daemons send today (Python side, verified)

- Arm: `write_reg(link, SX1276_REG_OP_MODE=0x01, SX1276_OPMODE_LORA_RXCONT=0x85)` = `REG_WRITE_REQ` 0x31, payload `[0x01, 0x85]`, ack `REG_WRITE_ACK_URC` 0xB1, preceded by a `REG_READ_REQ` pre-check at every call site. Defined at `method_h_stage2_tx_probe_v2.py:335-339`, constants `:207,:213`.
- Call sites: base `image_rx_daemon.py:441-452` (connect), `:540-557` (`_ensure_rxcont`, called `:666`, `:1076`), `:876-883` (after profile switch, inside `_apply_profile` — NOT warn-and-continue, see §1); tractor `firmware/tractor_x8/image_tx_daemon.py:606-609` (connect), `:719-736` (`_ensure_rxcont`, called `:644, :849, :1131, :1306`); probe autowake `method_h_stage2_tx_probe_v2.py:1466-1478`; `w2_02_radio_wake_rxcont.py:83`.
- Sleep cleanup: `radio_sleep.py` writes `[0x01, 0x80]`.
- Bench-only diag writes to reg 0x01: SPI-write isolation 0x81/0x83 (`method_h_stage2_tx_probe_v2.py:2514/:2522/:2531`) **and T2 TCXO test 0x00 (FSK SLEEP) / 0x01 (FSK STDBY) / 0x80 restore (`:1047/:1059/:1071`; same in `method_h_stage2_tx_probe.py:449/:461/:473`)**.
- The H7 never sends REG_WRITE — the constant exists only in the two `mh_wire.h` copies (`firmware/tractor_h7/murata_host/mh_wire.h:22`, `firmware/tractor_h7/src/murata_host/mh_wire.h:22`); no H7 .c/.cpp references it.

So **production** traffic to reg 0x01 is exactly three values: **0x80 (SLEEP), 0x81 (STANDBY, probe restore), 0x85 (RXCONT)** — all LoRa-mode (bit7 set). Bench diag additionally sends 0x00, 0x01, 0x83.

## 3. Options and recommendation

**Option A — dedicated host op** (e.g. `HOST_TYPE_RADIO_MODE_REQ` 0x32, routed through `sx1276_rx_arm()`/`sx1276_modes_to_sleep()`):
- Pros: goes through the real mode state machine — W1-9 opmode retry loop + `RADIO_OPMODE_DRIFT` fault (`sx1276_modes.c:107-127`), CRC-enabled guard (`sx1276_rx.c:68-72`); semantically first-class; no raw-write/sync dance.
- Cons: new wire type ⇒ lockstep churn across `host_types.h` (+ uniqueness switch `:220-252`), **both** `mh_wire.h` copies + `check_mh_wire_sync.py` SYNC_KEYS (`tools/check_mh_wire_sync.py:28,38`) + regenerated `tools/mh_wire_constants.py`, `method_h_stage2_tx_probe_v2.py`, and 5+ call sites across two daemons that each need a fallback for old firmware — `HostLink.request()` raises RuntimeError on ERR_PROTO (`method_g_stage1_probe.py:327-330`), so the fallback is a fast try/except-and-retry-raw, cheap but must be written at every site (the bench memory notes the deployed tree is already stale — mixed-version risk is real). And Batch-2 F1–F3 will move arming authority into firmware anyway, so a new host-arming op is churn that F2/F3 then re-obsoletes.

**Option B — always-on production gate on the existing write path (RECOMMENDED)**: RegOpMode writes with a value-allowlist `{0x80, 0x81, 0x85}` are accepted regardless of the diag flag; the RS-4.12 sync hook at `host_cmd.c:594` (unchanged) becomes reachable in flag=0 builds; the diag flag shrinks to gating only the other 12 registers and non-listed opmode values (which keeps T2's 0x00/0x01 and the SPI-isolation 0x83 working in diag builds, since 0x01 stays in the 13-address switch).
- Zero wire-protocol change ⇒ zero Python/H7 changes; works against every deployed daemon version.
- The value gate closes the real reason the flag must be conservative on this register: raw 0x83 (TX) would key the PA bypassing LBT/airtime/dwell accounting; FSK-mode values (bit7=0) are also refused. The remaining diag registers (0x1D/0x1E etc., the airtime-invariant bypass of `SETTINGS_REFERENCE.md:316`) go back behind a flag that can finally be 0 in production.
- Known deficit vs A (mark it in the commit): the raw path has no W1-9 retry — a TCXO-wedged chip silently ignores the write. Mitigation today is structural, not per-write: only the rx-daemon connect autowake (`image_rx_daemon.py:446`) and probe autowake read back post-write; both `_ensure_rxcont`s write blind but re-read opmode on every idle/post-TX cycle, so an ignored write is retried continuously. An optional follow-up ("routed-B": inside `handle_reg_write`, value 0x85 → `sx1276_rx_arm()`, 0x80 → `sx1276_modes_to_sleep()`, 0x81 → `sx1276_modes_to_standby()`) buys the retry+fault behavior with still no wire change — but it alters bench-measured timing (up to ~5 ms blocking retry per `sx1276_modes.c:108`, DIO/RF-switch writes) and should be a separate, bench-A/B'd commit, not part of F9 landing mid-campaign.

## 4. Exact changes (Option B, minimal)

| File | Change |
|---|---|
| `firmware/murata_l072/include/host_reg_gate.h` (NEW) | `bool host_reg_write_allowed(uint8_t reg_addr, uint8_t value, bool diag_enabled);` — pure, no deps (pattern of `sx1276_rx_retune_policy.c`). |
| `firmware/murata_l072/host/host_reg_gate.c` (NEW) | Implementation: `reg==0x01 && value ∈ {0x80,0x81,0x85}` → true unconditionally; else `diag_enabled &&` the existing 13-address switch (moved verbatim from `host_cmd.c:521-538`). |
| `firmware/murata_l072/host/host_cmd.c` | `handle_reg_write()` `:577`: call `host_reg_write_allowed(frame->payload[0], frame->payload[1], HOST_ALLOW_REG_WRITE_DIAG != 0)`; delete static `reg_write_allowed()` (`:519-543`). Sync hook `:594-596` untouched. |
| `firmware/murata_l072/config.h:61` | Wrap `HOST_ALLOW_REG_WRITE_DIAG` in `#ifndef` (today it is a bare `#define … 1`; an external `-D` override is a fatal macro redefinition under `-Werror` — same fix already applied to `HOST_EMIT_RADIO_IRQ_DEBUG_URC` at `:65-67`). Document the override as `make EXTRA_CFLAGS="-DHOST_ALLOW_REG_WRITE_DIAG=0"` — `EXTRA_CFLAGS` feeds the compile at `Makefile:186`; a command-line `CFLAGS=` would clobber the whole flag set (`Makefile:106` is `:=`). **Do not flip the default to 0 in the same commit** — separate commit after a bench soak (method_h T2 writes 0x00/0x01 and SPI-isolation writes 0x83, plus registers 0x40/0x33/… , all need diag builds). |
| `firmware/murata_l072/Makefile` | Add `host/host_reg_gate.c` to `SRCS` (host block `:57-65`); add `REG_WRITE_GATE_BIN` variable (near `:244`); add bench target `check-reg-write-gate` (links only `bench/host_proto/reg_write_gate.c` + `host/host_reg_gate.c`); append to the `check` line (`Makefile:246`, 25 → 26 targets) **and to `.PHONY` (`:146`)**. Bench recipes hardcode their own flags (no `$(CFLAGS)`), so the test drives both `diag_enabled` values via the function argument, not via `-D`. |
| `bench/host_proto/reg_write_gate.c` (NEW) | Test binary, cases in §6. |
| Docs | `SETTINGS_REFERENCE.md:316` (flag row semantics) and `:420` ("complete set" allowlist row is no longer complete — 0x01 is now value-gated outside the flag); `CONTROL_PLANE_DESIGN.md:418` (§10 F9 row); `TODO.md` RS-4.12 note (~`:1064-1071`). |

New structs: none. New registers: none (same RegOpMode 0x01; only the acceptance policy changes). New wire types/URCs: none. RAM: 0 bytes.

## 5. What could break (checked against the actual link sets)

- **Bench link sets — clean.** No bench target compiles `host_cmd.c` or `sx1276_modes.c` (verified against every `$(HOST_CC)` recipe in the Makefile), so no stub additions are needed in `sx1276_stub.c`/`fhss_stub.c`. The new `host_reg_gate.c` is dependency-free. Trap to avoid for the routed-B follow-up: `tx_len_guard.c:53` already defines a local `sx1276_rx_arm` stub (`void sx1276_rx_arm(uint8_t)` — signature doesn't even match the real `bool sx1276_rx_arm(void)`), so a future `sx1276_rx_arm` stub must NOT go into `sx1276_stub.c` (duplicate symbol in `check-tx-len-guard`, which links `sx1276_stub.c` + real radio objects — same collision rule documented at `sx1276_stub.c:23-28` and `fhss_stub.c:1-11`).
- **Static asserts:** `include/static_asserts.c` (27 lines) references none of these symbols — unaffected.
- **Wire parsers / Python mirrors:** unchanged. `REG_WRITE_ACK_URC` shape identical; `ERR_PROTO FORBIDDEN` still returned for refused registers. `base_station/lora_proto.py` carries no REG_WRITE mirror; both `mh_wire.h` copies, `tools/mh_wire_constants.py`, `tools/check_mh_wire_sync.py` untouched.
- **Daemons:** no behavior change with flag=1 builds; with flag=0 builds they *start working* (that's the point) — including the `_apply_profile` revert bug in §1. `radio_sleep.py` (0x80) keeps working. Bench probes writing 0x00/0x01/0x83/0x40 need diag builds after any future default flip — flagged above.
- **CI ownership guards:** `check-opmode-owner` (`Makefile:261` Windows / `:285` POSIX) greps `radio/*.c` only — the new `host/host_reg_gate.c` is invisible to it. `check-cfg-wire-owner` (`:265`) greps `host/*.c` for CFG URC types the new file never mentions — unaffected.
- **Capability bitmap:** `HOST_CAPABILITY_BITMAP=0x7F` (`host_types.h:17`) still advertises REG_IO regardless of flag (`SETTINGS_REFERENCE.md:428` already documents that quirk); B makes the advertisement partially true always. Doc note only.

## 6. Test additions

New bench target `check-reg-write-gate` (host gcc, pure, `make check`):
- diag=false: (0x01,0x85)/(0x01,0x80)/(0x01,0x81) → allowed; (0x01,0x83 TX) refused; (0x01,0x87 CAD), (0x01,0x86 RXSINGLE) refused; **(0x01,0x00 FSK-SLEEP) and (0x01,0x01 FSK-STDBY) refused — these are values the T2 probe really sends**; (0x01,0x05 FSK-RXCONT, bit7 clear) refused; (0x1D,\*), (0x1E,\*), (0x40,\*), (0x00,\*), (0xFF,\*) refused.
- diag=true: each of the exact 13 legacy addresses {0x01,0x06,0x07,0x08,0x09,0x1D,0x1E,0x26,0x31,0x33,0x37,0x3B,0x40} allowed with arbitrary values — this pins that T2 (0x00/0x01) and SPI-isolation (0x83) keep working in diag builds, and pins the list so drift is loud, mirroring `SETTINGS_REFERENCE.md:420`; a non-listed address (0x02) refused.
- Exhaustive property: all 256 addresses × value sample; diag=false ⇒ allowed iff `reg==0x01 && val∈{0x80,0x81,0x85}`.

Hardware verification (not `make check`): build once with `EXTRA_CFLAGS=-DHOST_ALLOW_REG_WRITE_DIAG=0`, run the run-31-style check — daemon connect → profile activation → confirm `write_reg(0x01,0x85)` acks, `rx_frames` climbs across firmware TXes, a 0x1D write answers FORBIDDEN, **and a profile switch completes without triggering the `_apply_profile` revert path**.

## 7. Flash cost

Minimal B: ≈ +64–128 B .text (one pure function with a switch + value compare; the moved allowlist switch is net-zero), 0 .bss/.data — against 38326 B used of 192K. Routed-B variant ≈ +100–200 B more. Option A would be ≈ +300–400 B (handler + dispatch case + ack). All negligible; measure exact delta with `arm-none-eabi-size` pre/post. Uncertainty: figures are -Os estimates, not measured.

## 8. Explicit uncertainties

- Whether any out-of-tree bench script writes opmode values outside {0x80,0x81,0x85} in a production (flag-default) context — repo grep found the diag probes noted in §2 (including T2's 0x00/0x01) and nothing else; deployed-tree drift (per bench memory) can't be ruled out, which is another reason to keep the flag default at 1 for one soak cycle.
- Routed-B's ≤5 ms blocking retry inside `handle_reg_write` is asserted safe from code reading (mode apply already busy-waits in TX context), not bench-measured.
- Flash figures are estimates, not measured builds.

Key files: `C:/Users/dorkm/Documents/GitHub/LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c`, `.../firmware/murata_l072/config.h`, `.../firmware/murata_l072/radio/sx1276_modes.c`, `.../firmware/murata_l072/host/host_cfg_profile.c`, `.../firmware/murata_l072/radio/sx1276_tx.c`, `.../firmware/murata_l072/radio/sx1276_rx.c`, `.../firmware/murata_l072/Makefile`, `.../firmware/murata_l072/bench/host_proto/tx_len_guard.c`, `.../base_station/image_rx_daemon.py`, `.../firmware/tractor_x8/image_tx_daemon.py`, `.../firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py`, `.../firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py`, `.../SETTINGS_REFERENCE.md`, `.../CONTROL_PLANE_DESIGN.md`.