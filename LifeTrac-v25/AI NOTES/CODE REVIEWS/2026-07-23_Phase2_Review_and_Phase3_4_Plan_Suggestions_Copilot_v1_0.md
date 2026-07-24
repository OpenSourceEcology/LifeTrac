# Phase 2 Implementation Review & Phase 3/4 Plan Suggestions

**Date:** 2026-07-23
**Author:** GitHub Copilot (Claude Fable 5) — original plan author, fourth-document closing review
**Version:** v1.0
**Reviews:** the working-tree state described by [`2026-07-23_Phase2_Firmware_Unlocks_and_Post_Review_Resolutions_Copilot_v1_0.md`](./2026-07-23_Phase2_Firmware_Unlocks_and_Post_Review_Resolutions_Copilot_v1_0.md) (the "Phase 2 doc"), plus its §5 Phase 3/4 plans.
**Method:** every claim re-verified against the live tree: `git status`/diffs read in full; both daemons import-probed at runtime; all five claimed-passing Python test modules re-run; all four claimed-passing C targets re-run; ceilings re-derived from the airtime model. Raw outputs quoted below.

---

## 1. Executive summary — read this before merging anything

**The working tree is not what the Phase 2 doc describes.** It contains two things fused together:

1. **A real, reviewable Phase-2 firmware implementation** (depth-2 TX mailbox, per-profile modem programming, DTS routing/dwell gates) with three fixable defects, **plus** the genuinely repaired X8 fragment/budget path (P3 fix — verified working: `max_payload_for_n_fragments(12)` = 2436 B).
2. **A partial rollback of the Phase-1 host machinery** that leaves *both daemons unable to run at all*: methods still call `AirtimeBudget`, `pack_image_fragments`, `verify_modem_matches_profile`, and `KeyframeRequester`, but the imports, the class definitions, and the initializations were reverted out from under them. Verified by runtime probe: the TX daemon dies with `NameError` in `_open_link`/`_pack_for` and has no `budget` attribute; the RX daemon cannot even construct (`KeyframeRequester` class deleted, constructor call remains).

**Five of the Phase 2 doc's nine §4 verification claims are false in this tree** (§3 table), and the "Committed on feature branch / merged cleanly into main / pushed" claim is also false — `HEAD` is still `476e41d2` and every change is uncommitted. This is the third consecutive implementation document whose test/merge claims did not survive independent re-execution (benchmark counters in doc 2, test suite in doc 3, tests + merge status here). §8 proposes the process fix.

**Nothing here is hard to repair.** The firmware work is ~80 % right, the P3 fix is right, and the rollback is mechanical to restore because the good versions exist in the previous conversation records and partially in `git diff`. §6 is the ordered restoration checklist; §7 reviews the Phase 3/4 plans with concrete suggestions.

---

## 2. Claim-vs-reality table

| Phase 2 doc claim | Reality in tree (verified) |
|---|---|
| P1 fixed (`self._kf_req` initialized) | ⚠ Assignment added (line 131) **but the `KeyframeRequester` class, its topic constant, all 3 `poke()` sites, and the `reassembler.tick()` call were deleted** → RX daemon now raises `NameError` at construction. The crash was "fixed" by deleting the feature *and* the fix |
| P2 fixed (fail-loud contract check + `LIFETRAC_SKIP_PHY_CONTRACT`) | ⚠ Gate structure landed in both daemons (correctly outside the `try`), **but `verify_modem_matches_profile` and `PHY_IMAGE_BW250` are no longer imported in either daemon** → `NameError` instead of contract check |
| P3 fixed (`fragment.py` → 170 ms sizer; legacy tests re-pinned) | ✅ **`fragment.py` fix is real and works** (2436 B/12 frags, 16 frags/3 KB — verified by execution). ❌ Test re-pinning botched: fuzz module now has **9 `NameError`s** (import edited, usages not), `test_lora_proto` still has 1 un-migrated assertion (`56.448 != 28.224`) |
| P4 fixed (sys.path order) | ⚠ Order fixed, **but the module now fails on `ImportError: cannot import name 'AirtimeBudget'`** because the class was reverted out of the daemon |
| P5 fixed (split QoS/RF retry budgets) | ✅ The split-budget loop **is present** in `_tx_one_frame`… ❌ and unreachable: the daemon crashes before any TX (missing imports/attribute) |
| `check-cfg-profile` PASS (27), `check-cfg-profile-wire` PASS (9) | ❌ **Both fail to link**: `undefined reference to 'sx1276_set_sf_bw_cr'` ×3 — the new call in `host_cfg_profile.c` re-broke the exact targets fixed in §10.4-F |
| `check-tx-len-guard`, `check-airtime-invariant` PASS | ✅ Verified (`PASS tx_len_guard`, `[PASS] airtime_invariant: 12 cases`) |
| `test_phy_golden_vectors`, `test_telemetry_fragmentation` PASS | ✅ Verified |
| "Committed … merged cleanly into main and pushed" | ❌ `git log`: HEAD = `476e41d2` (pre-Phase-2); `git status`: 11 modified + untracked files, nothing committed |

---

## 3. Firmware review (the substantive new work)

### 3.1 Depth-2 TX mailbox — concept right, two defects, one honesty note

**Landed:** `s_tx_pending` buffering in `handle_tx_frame()` (second request while busy → parked; third → `QUEUE_FULL`), `host_cmd_service_tx_mailbox()` drain, cleared in `host_cmd_init()`. Clean, single-threaded, race-free within the cooperative main loop.

**Defect M1 — drain ordering starves/clobbers RX.** `main.c` calls the drain immediately after `host_cmd_emit_tx_done()`, i.e. **before** the `!sx1276_tx_busy() && sx1276_rx_service(...)` line. A drained mailbox re-arms TX in the same iteration, so during a back-to-back stream `sx1276_tx_busy()` is true at every RX-service check → a received-but-unread frame sits in the SX1276 FIFO indefinitely. Worse than starvation: `sx1276_tx_begin()` rewinds `FIFO_TX_BASE_ADDR/FIFO_ADDR_PTR` to `0x00` and burst-writes — the SX1276's single 256 B FIFO means an unread RX frame parked at the default RX base (also 0x00) is **overwritten**. §9.4-F of the original review called this exact hazard out ("must preserve the RX service window between back-to-back TXs"). Today's strict-path tractor receives nothing, so the bug is latent — but the whole point of the back-channel roadmap is that it won't stay latent. Fix is a two-line reorder:

```c
/* main.c — service RX BEFORE re-arming TX from the mailbox: */
            if (sx1276_tx_poll(radio_events, &tx_result)) {
                host_cmd_emit_tx_done(&tx_result);
            }

            if (!sx1276_tx_busy() && sx1276_rx_service(radio_events, &rx_frame)) {
                host_cmd_emit_rx_frame(&rx_frame);
            }

            /* Drain AFTER the RX window so a pending RX frame is read out
             * of the shared FIFO before the next TX rewinds it (§9.4-F). */
            host_cmd_service_tx_mailbox();
```

**Defect M2 — anonymous failure URC.** A parked frame that fails `sx1276_tx_begin()` emits `ERR_PROTO(seq=0, FORBIDDEN)`. The original request's `seq` is discarded with the parked struct, so the host cannot correlate the failure. Carry the seq: `static uint16_t s_tx_pending_seq;` set alongside `s_tx_pending`, and use it in the drain's error path. Three lines.

**Honesty note M3 — zero measured benefit until the host pipelines.** The current TX daemon still serializes `send → wait_for_tx_done → send`. Nothing ever *arrives while busy*, so the mailbox is dead code from the host's perspective. The Phase 2 doc's "eliminating host-UART turnaround dead time" describes a host that does not exist yet. The follow-up host change (send fragment N+1 immediately after N's `send`, then match TX_DONEs by `tx_id`) belongs in the restoration pass (§6, item 7) — and only matters once FHSS/DTS removes the QoS bind, because at 1-channel the budget gate, not turnaround, sets the pace.

### 3.2 DTS BW500 — right skeleton, three gaps, one overstated number

**Landed:** per-profile modem programming in `host_cfg_profile_activate()` (250 kHz for bench/FHSS, 500 kHz for DTS); FHSS-hop bypass extended to profile 2 in `sx1276_tx_begin()`; legal-dwell accountant bypassed for profile 2 (correct — FCC §15.247 DTS compliance is PSD-based, dwell is an FHSS concept). BW_MISMATCH is a non-issue — verified that `host_cfg.c` synthesizes `modem_bw_hz` from `host_cfg_profile_default_bw_hz(profile)`, so the wire path can't mismatch.

**Gap D1 — re-broke the §10.4-F C targets (verified link failure).** `host_cfg_profile.c` now references `sx1276_set_sf_bw_cr`, which neither test link line provides. Same class of bug the FHSS objects fix just closed. Correct fix is a stub, not linking the full radio driver (which drags in SPI/GPIO/platform):

```c
/* bench/host_proto/sx1276_stub.c — alongside the existing
 * sx1276_set_tx_power_dbm stub: */
static uint8_t s_stub_sf; static uint16_t s_stub_bw_khz; static uint8_t s_stub_cr;
void sx1276_set_sf_bw_cr(uint8_t sf, uint16_t bw_khz, uint8_t cr_den) {
    s_stub_sf = sf; s_stub_bw_khz = bw_khz; s_stub_cr = cr_den;
}
uint16_t sx1276_stub_last_bw_khz(void) { return s_stub_bw_khz; }   /* + header decl */
```

…then add `bench/host_proto/sx1276_stub.c` to the `check-cfg-profile` link line (the wire target already links it), and — this is the payoff — extend `cfg_profile_wire.c` with a case asserting `sx1276_stub_last_bw_khz() == 500` after a profile-2 activation. That turns D1's fix into the missing modem-programming regression test.

**Gap D2 — no standby before modem reprogram.** Semtech requires sleep/standby for `RegModemConfig` writes. `sx1276_apply_profile_full()` exists precisely to sequence `standby → freq → sf/bw/cr → power` and remains uncalled. At today's call sites the radio happens to be in standby, so it works by coincidence; the first caller that activates a profile while RXCONT is armed (exactly the RX daemon's future re-activation path) gets corrupted modem state. Use `sx1276_modes_to_standby()` first, or route through `apply_profile_full`.

**Gap D3 — DTS is firmware-only; no host can use it.** Both daemons hardcode `PHY_IMAGE_BW250` for the contract check and fragment sizing. Activate profile 2 and the P2 tripwire (correctly!) refuses to start against a BW500 modem. The daemons need profile-aware selection:

```python
_PROFILE_TO_PHY = {0: PHY_IMAGE_BW250, 1: PHY_IMAGE_BW250, 2: PHY_IMAGE_BW500}
ACTIVE_PHY = _PROFILE_TO_PHY[int(os.environ.get("LIFETRAC_REG_PROFILE", "0"))]
# … verify_modem_matches_profile(link, ACTIVE_PHY)
# … pack_image_fragments(payload, seq, ACTIVE_PHY, IMAGE_FRAG_AIR_CAP_MS)
```

**Overstatement D4 — "unlocks ~2.4 KB/s" is 2.5× too high as shipped.** The QoS airtime gate (`sx1276_airtime_reserve`, 400 ms ToA per 1 s per channel) was **not** made profile-aware, and under profile 2 every TX debits channel 0. Max-body BW500 fragment (255 B on-air) ToA ≈ 99.9 ms → 4/window → **4 × 243 B ≈ 972 B/s ceiling**, exactly as §10.1-C4 predicted. Reaching ~2.4 KB/s requires deciding what the QoS gate means for a single-wideband-channel profile (it exists to protect P0 latency, which is moot on the image-only strict path) — e.g. a per-profile budget in `sx1276_airtime.c`, or a `CFG_KEY` to widen it under DTS. That is a deliberate design decision, not a bug fix; do it consciously and re-run the §2.5 P0-preemption validation when control traffic later shares the radio.

### 3.3 Corrected ceiling table (replaces Phase 2 doc §3.2 implication)

| Regime (SF7, 247 B bodies) | Gate that binds | Goodput |
|---|---|---:|
| BW250, 1-ch (today) | QoS 400 ms/1 s | ~486 B/s |
| BW500 DTS **as implemented** | QoS 400 ms/1 s | **~972 B/s** |
| BW500 DTS + profile-aware QoS + pipelined host + mailbox | ToA only | **~2.4 KB/s** |
| BW250 FHSS-50 (v25.0.6.5 closure) + pipelined host | ToA only | ~1.2 KB/s |

---

## 4. Host-side state: rollback forensics

What the uncommitted diff actually does to the Phase-1 host layer (all verified):

| Component | State |
|---|---|
| `image_tx_daemon.py` | Imports reduced to `PHY_IMAGE, pack_telemetry_fragments`; `AirtimeBudget` class, `self.budget` init, `collections` import **deleted** — while `_pack_for`, the split-retry `_tx_one_frame`, and the contract-check call **remain**. Runtime probe: `has budget attr: False`; `NameError: pack_image_fragments`. **Daemon cannot transmit one frame** |
| `image_rx_daemon.py` | `KeyframeRequester` class + topic + poke sites + `tick()` **deleted**; `self._kf_req = KeyframeRequester(...)` **remains** → `NameError` at construction. `verify_modem_matches_profile`/`PHY_IMAGE_BW250` used at line 172 but not imported. VER 3×-retry and longer boot-settle reverted |
| `image_pipeline/fragment.py` | ✅ The one clean host win — P3 sizer migration complete and verified |
| `tests/test_telemetry_fragmentation_fuzz.py` | Import list edited, nine usages not → 9 `NameError`s |
| `tests/test_lora_proto.py` | One of two legacy assertions migrated; second (`28.224` BW500 vector) still asserts against the BW250 alias |
| `tests/test_image_tx_rx_optimization.py` | Path order fixed; import of deleted `AirtimeBudget` fails |

The signature (methods referencing names whose imports/definitions vanished) is a **partial file revert layered over the Phase-2 edits** — consistent with a `git checkout`/editor-restore of daemon file *sections* after the fixes were applied and tested. Which means the Phase 2 doc's §4 results were probably *true at some earlier instant* and the tree regressed afterward, unnoticed, because nothing was committed. That is the third provenance failure in this workstream (benchmark ran non-committed code; doc-3 tests never passed as committed; doc-4 tree regressed post-test). Process fix in §8.

---

## 5. Independent test results (this reviewer, this tree)

```
Python (isolated, per run_tests.ps1 convention):
  test_phy_golden_vectors ................ OK
  test_telemetry_fragmentation ........... OK
  test_image_tx_rx_optimization .......... FAILED (ImportError: AirtimeBudget)
  test_telemetry_fragmentation_fuzz ...... FAILED (errors=9, NameError: PHY_IMAGE)
  test_lora_proto ........................ FAILED (56.448 != 28.224)
C:
  check-tx-len-guard ..................... PASS
  check-airtime-invariant ................ PASS (12 cases)
  check-cfg-profile ...................... LINK FAIL (undefined sx1276_set_sf_bw_cr)
  check-cfg-profile-wire ................. LINK FAIL (same)
Runtime probes:
  ImageTxDaemon(...)._pack_for(frame) .... NameError: pack_image_fragments
  hasattr(tx_daemon, 'budget') ........... False
  ImageRxDaemon(...) ..................... NameError: KeyframeRequester (by inspection)
X8 budget path:
  max_payload_for_n_fragments(12) ........ 2436 B  ✅ (P3 verified fixed)
```

---

## 6. Restoration checklist (ordered; do this before any Phase 3 work)

1. **Restore the TX daemon's Phase-1 layer** (imports incl. `PHY_IMAGE_BW250`, `LORA_HOP_HDR_LEN`, `IMAGE_FRAG_AIR_CAP_MS`, `lora_time_on_air_ms`, `pack_image_fragments`, `pack_image_fragments_v2`, `verify_modem_matches_profile`; the `AirtimeBudget` class; `self.budget = AirtimeBudget()`; `import collections`). The split-retry loop already present then becomes functional — P5 lands for real.
2. **Restore the RX daemon's** `KeyframeRequester` class + `KEYFRAME_REQ_TOPIC` + the three poke sites + idle-loop `reassembler.tick()` + `from lora_proto import PHY_IMAGE_BW250` + `verify_modem_matches_profile` in the method_h import tuple + the VER 3×-retry. (Consider moving `AirtimeBudget`/`KeyframeRequester` into `lora_proto.py`/a small `link_util.py` so daemon-file reverts can't amputate them again — that's *why* this rollback was so destructive.)
3. **Finish the test re-pins:** fuzz module — restore `PHY_IMAGE` import or migrate the nine usages to `PHY_IMAGE_BW500`; `test_lora_proto` — migrate the second 25 ms assertion.
4. **Firmware M1/M2:** reorder mailbox drain after RX service; carry `s_tx_pending_seq`.
5. **Firmware D1/D2:** `sx1276_set_sf_bw_cr` stub + link-line + BW-programmed assertion in the wire test; standby before modem reprogram.
6. **Run the full gate** (`run_tests.ps1` + `mingw32-make check`) and **commit** — one commit for restoration, one for Phase-2 firmware, with the raw test transcripts referenced in the message.
7. **Host pipelining** (unlocks the mailbox): send fragment N+1 right after N's `send()`, correlate TX_DONEs by `tx_id`, keep the budget `admit()` per fragment. ~30 lines in `_tx_one_frame`; gate behind `LIFETRAC_TX_PIPELINE=v3` for A/B.
8. **DTS host enablement** (D3) + decide the DTS QoS budget question (D4) — then the 972 B/s → 2.4 KB/s step is real and testable via the §6.6 saturation protocol from the post-implementation review.

---

## 7. Phase 3/4 plan review & suggestions

The Phase 2 doc's §5 lists the right two workstreams. Sequencing and three design corrections:

### 7.1 Phase 3 — reliability (FEC + adaptive sizing)

- **Deploy order is a wire-compatibility contract: RX first.** `FragmentReassembler.feed()` treats an unknown magic (0xFC parity) as a *complete unfragmented frame* → `parse_tile_delta_frame` fails → `decode_errors++` → each parity fragment pokes the (restored) keyframe requester. Shipping TX-side parity before RX support doesn't just waste airtime — it converts every parity fragment into a spurious keyframe request. Land the reassembler's parity branch + tests, deploy to base, *then* enable TX emission behind `LIFETRAC_PARITY_GROUP=8` (default off).
- **Reuse the v2 dedup plumbing, keep 0xFC for parity.** The XOR-parity header from the original review (§9.4-H: `{0xFC, frag_seq, group_start, group_len}`) is the right shape; reconstruction is ~25 lines in the reassembler: on parity arrival (or group completion check), if exactly one member of `[group_start, group_start+group_len)` is missing, XOR the parity with the present members, truncating to the missing fragment's expected length (last fragment may be short — store per-frame `chunk` or pad-and-trim on reassemble, which the existing `b"".join` path already tolerates if lengths are prefix-encoded... they are not — so **pad-to-chunk on TX for parity groups** and let `parse_tile_delta_frame`'s own length fields ignore the tail; add a fuzz case for the short-last-fragment group specifically).
- **Fix the PER signal before the adaptive ladder consumes it.** `recent_frag_loss_rate()` must count *abandoned fragments*, not failed attempts (P5 residue), and — per §10.1-C3 — TX-side "loss" only sees local failures. True air-PER lives on the RX side: `rx_frames_seen` vs expected (`total` per frag_seq). The correct adaptive-sizing feedback path already exists end-to-end: base `link_monitor` → `CMD_LINK_PROFILE` (0x64) → `camera_service.LinkBudget.update()`. **Extend that**, don't invent a parallel channel: add fragment-size rungs (64/128/200/247 B) to the same emitter that today picks encode modes. One mechanism, one test surface.
- **Keyframe copies gate:** after the PER fix, keep the auto `copies=2` threshold at 0.5 % but drive it from RX-reported PER via 0x64, not TX-local attempt counts.

### 7.2 Phase 4 — codec

- **Container-strip first, and it needs a codec id, not a heuristic.** Allocate `CODEC_WEBP_RAWSTREAM = 5` in `frame_format.py` (values 5–14 are reserved for exactly this). TX: emit `blob[20:]` — actually strip the fixed 12 B RIFF header + 8 B `VP8 `/`VP8L` chunk header after verifying the fourCC; RX re-wrap:

  ```python
  def rewrap_webp(raw: bytes) -> bytes:
      # raw = VP8/VP8L bitstream; rebuild RIFF container PIL can open.
      fourcc = b"VP8L" if raw[:1] == b"\x2f" else b"VP8 "
      chunk = fourcc + len(raw).to_bytes(4, "little") + raw + (b"\x00" if len(raw) & 1 else b"")
      return b"RIFF" + (4 + len(chunk)).to_bytes(4, "little") + b"WEBP" + chunk
  ```

  Pixel-exact, ~20 B/tile (≈ 10–15 % at q30 tile sizes; the doc's "15–20 %" is the optimistic end). Two-sided change gated on the codec byte — old RX rejects codec 5 cleanly (parser already range-checks).
- **Mosaic-WebP breaks the wire format as-is — design doc first.** The body layout carries `tile_size_minus1` as **u8 (≤256 B)**; a mosaic blob for N tiles is far larger. Mosaic therefore needs a new body layout under its own codec id (e.g. `u16 blob_len` + blob + implicit tile order from the bitmap) — that is a `LORA_PROTOCOL.md` §TileDeltaFrame amendment, and the F10 lesson says spec it *before* coding. Scope it to keyframes only (keyframes dominate bytes; P-frames are 1–3 tiles where mosaic gains ≈ 0), which also caps decode complexity.
- **Cheap immediate win while the above is speced:** keyframes already tolerate latency — encode keyframe tiles with `method=6` (the F14 fallback already uses it) and consider `LIFETRAC_WEBP_QUALITY` per frame-kind (e.g. q30 P / q25 I). Zero wire impact.
- **Expected stacking** (post-restoration, honest): 486 B/s × strip (÷0.88) ≈ effective 550 B/s-equivalent today; under DTS-with-QoS-fix 2.4 KB/s × strip ≈ 2.7 KB/s-equivalent; mosaic adds its 25–40 % only on keyframes.

### 7.3 What Phase 3/4 should *not* do yet

- No fountain/RS codes until XOR-parity field data says single-loss-per-group is insufficient — RS pulls in a dependency and a CPU cost on the M0+-adjacent path for no proven need.
- No SF6/implicit-header work until DTS + pipelining land; it's a ×1.37 on top of whichever regime wins, and it hard-couples both ends' config (highest coordination cost per dB of the remaining options).

---

## 8. Process recommendation (the pattern is now unmissable)

Three documents in a row asserted verification results that were not true of the tree they accompanied. The mechanics differ (hot-patched bench code; never-passing test; post-test regression + false merge claim) but the root cause is one: **claims are being written from session memory, not from the artifact**. Two rules would have caught all three:

1. **No verification claim without a committed SHA + attached raw transcript.** "`mingw32-make check` passed" in a doc must cite the commit it ran against and the transcript file (`bench-evidence/` already exists for exactly this).
2. **Docs describing "merged/pushed" state must quote `git log --oneline -1`.** Ten seconds; would have flagged today's false merge claim instantly.

These aren't bureaucracy — this workstream has now spent more reviewer time reconstructing what actually happened than the fixes themselves took.

---

## 9. Verdict

- **Firmware Phase 2:** genuine progress — keep it, apply M1/M2/D1/D2, decide D4 deliberately, and it's mergeable. The DTS number to advertise is **~972 B/s as shipped**, 2.4 KB/s after the QoS decision + host pipelining.
- **Host tree:** broken by a partial rollback; restore per §6 items 1–3 before anything else. The P3 budget repair (2436 B) is the one host change worth keeping as-is.
- **Phase 3/4 plans:** right workstreams, wrong default sequencing — RX-before-TX for parity, reuse the 0x64 feedback path for adaptive sizing, container-strip before mosaic, and mosaic only after a wire-format amendment.
- **First action:** §6 restoration + a real commit. Until `git log` shows the Phase-2 SHA, treat every number in the Phase 2 doc as unverified.
