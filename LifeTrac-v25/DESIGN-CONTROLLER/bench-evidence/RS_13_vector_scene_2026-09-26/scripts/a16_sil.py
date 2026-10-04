"""RS-13.1 anomaly A16 -- offline (SIL) reproduction, kept with the RS-13.1 record.

Drives the REAL VectorEncoder (firmware/tractor_x8/x8_image_pipeline/encode_vector.py)
into the REAL VectorSceneStore (base_station/image_pipeline/vector_scene_store.py)
through the TileDeltaFrame container, like base_station/tests/test_vector_sync.py.

VS1 has no uplink, so the encoded stream does not depend on what the base
receives: each stream is encoded once, replayed loss-free into a store, and at
every drop point the store is forked (deepcopy) and fed the next WINDOW frames
with that one frame lost.

Cases per budget (243 B = image_bw500 / DTS profile 2, 203 B = image_bw250 / FHSS profile 1):
  real              encoder as shipped (60 s safety refresh)
  norefresh         SAFETY_REFRESH_S = 1e9 in THIS process: does the base recover
                    without an epoch start?
  norefresh+keepdel replay counterfactual: the lost frame's DEL records still
                    arrive (DEL-only body): is the lost DEL (base ghost) the trigger?
  norefresh+confirm store counterfactual: a CONFIRM whose tag matches verifies even
                    in resync / under a mismatching DIGEST (the TTL cascade's gate)
  floor             encoder counterfactual: on a carousel-due frame (kappa 0.25 duty)
                    slots 5-6 leave room for the slot-7 records the kappa cap takes
  floorall          floor + carousel due on EVERY frame (25 % of every frame)
Nothing in the code tree is modified; patches live only in this process.

Usage: PYTHONIOENCODING=utf-8 py -3 a16_sil.py [--frames 400] [--step 3]
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import statistics
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a16_common import (CH, CW, VectorSceneStore, encode_stream, ev, vs)  # noqa: E402

SCENE = dict(n_star=8, n_static=12, n_static_trees=5)
WINDOW = 70              # frames observed after each drop (35 s at 2 fps)
A16_MISSING_RUN = 10     # frames the base lacks an encoder-live shape (5 s at 2 fps) to call it A16-like


# ---------------------------------------------------------------- counterfactual patches (this process only)

def pack_with_floor(cands, total_bits, reserve, carousel_bits):
    """encode_vector._pack with a carousel floor: the bits the kappa cap would
    give slot 7 on a carousel-due frame are held back from slots 5-6.
    Otherwise identical to the shipped greedy packer."""
    cands.sort(key=lambda c: (c.slot, c.order))
    floor = 0
    if carousel_bits:
        for c in cands:
            if c.slot == 7 and floor + c.bits <= carousel_bits:
                floor += c.bits
    used = car_used = 0
    packed, packed_ids = [], set()
    for c in cands:
        if c.slot < 5:
            limit = total_bits
        elif c.slot < 7:
            limit = total_bits - reserve - floor
        else:
            limit = total_bits - reserve
        if used + c.bits > limit or (c.needs is not None and id(c.needs) not in packed_ids):
            continue
        if c.slot == 7:
            if car_used + c.bits > carousel_bits:
                continue
            car_used += c.bits
        packed.append(c)
        packed_ids.add(id(c))
        used += c.bits
    return packed, used


def floor_patch(enc):
    enc._pack = pack_with_floor            # instance attribute: called as self._pack(cands, ...)


def floorall_patch(enc):
    enc._pack = pack_with_floor
    enc._carousel_due = lambda level: ev.LEVEL_KAPPA[level] > 0


class ConfirmInResyncStore(VectorSceneStore):
    """Store counterfactual: a matching CONFIRM tag verifies whatever the
    DIGEST / resync state (the shipped store refuses it, vector_scene_store.py
    _apply_confirm: ``elif self._digest_ok is not False and not self._resync``)."""

    def _apply_confirm(self, rec, clk):
        for i, tag in enumerate(rec.tags):
            if tag is None:
                continue
            self._items += 1
            sh = self._lookup(rec.base_id + i)
            if sh is None:
                self._orphan()
                continue
            if (self._state_hash(sh) & 3) != tag:
                self._orphan()
            else:
                self._verify(sh, clk)


# ---------------------------------------------------------------- replay

def _row(store, fr, lost, prev):
    st = store.stats
    base = {sh.id: (sh.dhash, store._state_hash(sh)) for sh in store._live()}
    enc = {i: (v[0], v[1]) for i, v in fr["live"].items()}
    return dict(
        i=fr["i"], lost=lost, key=fr["key"], digest_ok=None if lost else store._digest_ok,
        resync=st["resync"], orph=st["orphans"] - prev["orphans"], ttl=st["ttl_dropped"],
        resync_in=st["resync_events"] - prev["resync_events"],
        resync_digest_end=st["resync_digest_ends"] - prev["resync_digest_ends"],
        base_n=len(base), enc_n=len(enc),
        missing=sorted(set(enc) - set(base)), ghosts=sorted(set(base) - set(enc)),
        stale=sorted(i for i in set(enc) & set(base) if enc[i] != base[i]),
        in_step=(base == enc) and not st["resync"]), st


def _feed(store, fr, rx, lost, keep_dels):
    if not lost:
        res = store.ingest(fr["body"], fr["frame_kind"], rx, 0.0)
        assert res.applied, (fr["i"], res)
        return
    if keep_dels and not fr["key"]:
        dec = vs.decode_frame(fr["body"], fr["frame_kind"])
        dels = [r for r in dec.records if isinstance(r, vs.Del)]
        if dels:
            body = vs.encode_frame(dec.header, dels, len(fr["body"]))
            res = store.ingest(body, fr["frame_kind"], rx, 0.0)
            assert res.applied, (fr["i"], res)


def forked_replay(stream, drops, store_cls=VectorSceneStore, keep_dels=False):
    """Loss-free main replay; at each drop point fork the store and feed the
    next WINDOW+1 frames with the first one lost. Returns (main rows, {d: window rows})."""
    store = store_cls(CW, CH)
    rx = 10_000
    prev = store.stats
    main_rows, forks = [], {}
    todo = set(drops)
    for fr in stream:
        rx += 500
        if fr["i"] in todo:
            fork = copy.deepcopy(store, {id(store._lock): threading.RLock()})
            fprev, frx, rows = prev, rx, []
            for k, ffr in enumerate(stream[fr["i"]:fr["i"] + 1 + WINDOW]):
                _feed(fork, ffr, frx, k == 0, keep_dels)
                row, fprev = _row(fork, ffr, k == 0, fprev)
                rows.append(row)
                frx += 500
            forks[fr["i"]] = rows
        _feed(store, fr, rx, False, False)
        row, prev = _row(store, fr, False, prev)
        main_rows.append(row)
    return main_rows, forks


# ---------------------------------------------------------------- analysis

def episode(stream, d, rows, base_ttl_before):
    win = rows[1:]                                         # frames after the lost one
    lost_fr = stream[d]
    prev_live = set(stream[d - 1]["live"]) if d > 0 else set()
    rec = None
    for k in range(len(win) - 2):
        if all(r["in_step"] for r in win[k:k + 3]):
            rec = k + 1                                   # frames after the drop
            break
    keys = [k + 1 for k, r in enumerate(win) if r["key"]]
    first_key = keys[0] if keys else None
    waited = bool(rec is not None and first_key is not None and rec >= first_key
                  and not any(r["in_step"] for r in win[:first_key - 1]))
    run = best = 0
    for r in win:
        run = run + 1 if r["missing"] else 0
        best = max(best, run)
    miss_rows = [r for r in win if r["missing"]]
    end = rec if rec is not None else len(win)
    seg = stream[d + 1:d + 1 + end]
    due = [f for f in seg if f["car_bits"]]
    resynced = any(r["resync_in"] for r in rows)
    ended_by = None
    if resynced:
        if any(r["resync_digest_end"] for r in rows):
            ended_by = "digest"
        elif not rows[-1]["resync"]:
            ended_by = "epoch_start"
        else:
            ended_by = "not_ended"
    went_missing = {}
    for k, r in enumerate(win):
        for i in r["missing"]:
            went_missing.setdefault(i, k)
    resent = {i: next((k + 1 for k in range(k0 + 1, len(win)) if i in stream[d + 1 + k]["defines"]), None)
              for i, k0 in went_missing.items()}
    # Why is a missing id not repaired? For every (frame, id) the base lacks
    # while the encoder counts it live, classify what the NEXT frame did with it.
    why = {"carousel_packed": 0, "define_sent_other": 0, "carousel_offered_not_packed": 0,
           "repeat_pending_not_packed": 0, "upd_or_ucol_no_carousel": 0, "confirm_tag_only": 0,
           "released_by_encoder": 0, "nothing_named": 0}
    for k, r in enumerate(win):
        if d + 2 + k >= len(stream) or (rec is not None and k + 1 >= rec):
            break
        f = stream[d + 2 + k]
        for i in r["missing"]:
            if i in f["defines"]:
                why["carousel_packed" if i in f["car_packed_ids"] else "define_sent_other"] += 1
            elif i in f["car_offered_ids"]:
                why["carousel_offered_not_packed"] += 1
            elif i in f["live"] and f["live"][i][4] > 0:
                why["repeat_pending_not_packed"] += 1
            elif i in f["upd_ids"]:
                why["upd_or_ucol_no_carousel"] += 1
            elif i in f["confirm_ids"]:
                why["confirm_tag_only"] += 1
            elif i not in f["live"]:
                why["released_by_encoder"] += 1
            else:
                why["nothing_named"] += 1
    a16 = best >= A16_MISSING_RUN and (rec is None or waited)
    return dict(
        drop=d, lost_key=lost_fr["key"], lost_dels=sorted(lost_fr["dels"]),
        lost_fresh=sorted(lost_fr["defines"] - prev_live), lost_bytes=lost_fr["bytes"],
        disturbed=not all(r["in_step"] for r in win[:1]), recovered_after=rec,
        first_key_after=first_key, waited_for_epoch=waited,
        resync=resynced, resync_ended_by=ended_by, missing_max_run=best,
        orph_per_missing_frame=(statistics.mean(r["orph"] for r in miss_rows) if miss_rows else 0.0),
        ghost_max=max((len(r["ghosts"]) for r in rows), default=0),
        ghost_frames=sum(1 for r in win if r["ghosts"]),
        base_ttl_in_window=rows[-1]["ttl"] - base_ttl_before,
        enc_ttl_in_window=stream[min(len(stream) - 1, d + WINDOW)]["ttl_dropped_enc"] - stream[d - 1]["ttl_dropped_enc"],
        car_due_frames=len(due), car_due_zero=sum(1 for f in due if f["car_packed"] == 0),
        car_records=sum(f["car_packed"] for f in seg),
        missing_ids_resent_after=resent, missing_id_frames_why=why, a16_like=a16,
    )


def pct(xs, q):
    if not xs:
        return None
    s = sorted(xs)
    return s[min(len(s) - 1, max(0, round((len(s) - 1) * q)))]


def summarise(name, stream, budget, eps, main_rows):
    by = [f["bytes"] for f in stream[5:]]
    due = [f for f in stream[5:] if f["car_bits"]]
    dist = [e for e in eps if e["disturbed"]]
    recs = [e["recovered_after"] for e in dist if e["recovered_after"] is not None]
    a16 = [e for e in eps if e["a16_like"]]
    bins = {"1-3": (1, 3), "4-10": (4, 10), "11-20": (11, 20), "21-25": (21, 25), "26-40": (26, 40),
            "41-70": (41, 70)}
    return dict(
        case=name,
        frame_bytes=dict(min=min(by), p50=pct(by, 0.5), max=max(by),
                         ge_cap_minus7=f"{sum(b >= budget - 7 for b in by)}/{len(by)}"),
        records_p50=pct([f["n_records"] for f in stream[5:]], 0.5),
        n_live_p50=pct([f["n_live"] for f in stream[5:]], 0.5),
        residual_p50=round(pct([f["residual"] for f in stream[5:]], 0.5), 1),
        slot5_unpacked_mean=round(statistics.mean(f["slot5_unpacked"] for f in stream[5:]), 2),
        epoch_starts=[(f["i"], f["trigger"]) for f in stream if f["key"] and f["committed"]],
        lossfree_out_of_step_frames=sum(1 for r in main_rows[1:] if not r["in_step"]),
        carousel=dict(due_frames=len(due), due_with_zero_packed=sum(1 for f in due if f["car_packed"] == 0),
                      records_per_due_frame_mean=round(statistics.mean(f["car_packed"] for f in due), 2) if due else None,
                      offered_per_due_frame_mean=round(statistics.mean(f["car_offered"] for f in due), 2) if due else None),
        drops=len(eps), drops_lost_frame_had_del=sum(1 for e in eps if e["lost_dels"]),
        disturbed=len(dist), recovered_within_window=f"{len(recs)}/{len(dist)}",
        recovery_frames=dict(p10=pct(recs, 0.1), p50=pct(recs, 0.5), p90=pct(recs, 0.9),
                             max=max(recs) if recs else None,
                             hist={b: sum(1 for x in recs if lo <= x <= hi) for b, (lo, hi) in bins.items()},
                             unrecovered=len(dist) - len(recs)),
        resync_ended_by={k: sum(1 for e in eps if e["resync_ended_by"] == k) for k in ("digest", "epoch_start", "not_ended")},
        ghost_episodes=sum(1 for e in eps if e["ghost_max"]),
        a16_like=f"{len(a16)}/{len(eps)}",
        a16_like_drops=[e["drop"] for e in a16],
        a16_waited_for_epoch=sum(1 for e in a16 if e["waited_for_epoch"]),
        a16_unrecovered_in_window=sum(1 for e in a16 if e["recovered_after"] is None),
        a16_lost_frame_had_del=sum(1 for e in a16 if e["lost_dels"]),
        a16_missing_run=dict(p50=pct([e["missing_max_run"] for e in a16], 0.5),
                             max=max((e["missing_max_run"] for e in a16), default=None)),
        a16_orph_per_missing_frame=round(statistics.mean(e["orph_per_missing_frame"] for e in a16), 2) if a16 else None,
        a16_car_due_frames_zero_packed=f"{sum(e['car_due_zero'] for e in a16)}/{sum(e['car_due_frames'] for e in a16)}",
        a16_base_vs_enc_ttl_drops=f"{sum(e['base_ttl_in_window'] for e in a16)} vs {sum(e['enc_ttl_in_window'] for e in a16)}",
        a16_missing_ids_never_resent=sum(1 for e in a16 for v in e["missing_ids_resent_after"].values() if v is None),
        a16_missing_ids_total=sum(len(e["missing_ids_resent_after"]) for e in a16),
        a16_missing_id_frames_why={k: sum(e["missing_id_frames_why"][k] for e in a16)
                                   for k in (a16[0]["missing_id_frames_why"] if a16 else {})},
    )


def trace(rows, stream, before=None):
    lines = []
    for r in rows:
        f = stream[r["i"]]
        dig = "-" if r["digest_ok"] is None else ("ok" if r["digest_ok"] else "BAD")
        lines.append(f"#{r['i']:3d} {'LOST' if r['lost'] else '    '} key={int(r['key'])} {f['bytes']:3d}B "
                     f"dig={dig:3s} enc/base n_live=({r['enc_n']},{r['base_n']}) orph+{r['orph']} ttl={r['ttl']} "
                     f"rs={int(r['resync'])} car={f['car_packed']}/{f['car_offered']}{'*' if f['car_bits'] else ' '} "
                     f"missing={r['missing']} ghosts={r['ghosts']} stale={len(r['stale'])}")
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=400)
    ap.add_argument("--step", type=int, default=3)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "a16_sil_out"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    t0 = time.time()
    print(f"encoder module: {ev.__file__}")
    print(f"scene {SCENE}, frames {a.frames} (2 fps), quality 80 (V0), drop every {a.step} from frame 30, "
          f"window {WINDOW}, TTL_FRAMES {vs.TTL_FRAMES}, LEVEL_KAPPA {ev.LEVEL_KAPPA}")
    summary = {}
    drops = list(range(30, a.frames - WINDOW - 1, a.step))
    for bname, budget in (("bw500", 243), ("bw250", 203)):
        streams = {}
        for sname, kw in (("real", {}), ("norefresh", dict(safety_s=1e9)),
                          ("floor", dict(safety_s=1e9, patch=floor_patch)),
                          ("floorall", dict(safety_s=1e9, patch=floorall_patch))):
            streams[sname] = encode_stream(a.frames, budget, scene_kw=SCENE, **kw)
        cases = (("real", "real", VectorSceneStore, False),
                 ("norefresh", "norefresh", VectorSceneStore, False),
                 ("norefresh+keepdel", "norefresh", VectorSceneStore, True),
                 ("norefresh+confirm", "norefresh", ConfirmInResyncStore, False),
                 ("floor(norefresh)", "floor", VectorSceneStore, False),
                 ("floorall(norefresh)", "floorall", VectorSceneStore, False))
        for cname, sname, store_cls, keep in cases:
            t1 = time.time()
            stream = streams[sname]
            main_rows, forks = forked_replay(stream, drops, store_cls, keep)
            eps = [episode(stream, d, forks[d], main_rows[d - 1]["ttl"]) for d in drops]
            s = summarise(f"{bname}/{cname}", stream, budget, eps, main_rows)
            s["encode_ms_p50"] = round(pct([f["ms"] for f in stream], 0.5), 1)
            s["replay_s"] = round(time.time() - t1, 1)
            summary[s["case"]] = s
            print(json.dumps(s, default=str), flush=True)
            tag = cname.replace("+", "_").replace("(", "_").replace(")", "")
            with open(os.path.join(a.out, f"episodes_{bname}_{tag}.json"), "w", encoding="utf-8") as fh:
                json.dump(eps, fh, default=str, indent=1)
            a16 = [e for e in eps if e["a16_like"]]
            pick = a16[0]["drop"] if a16 else max(eps, key=lambda e: e["recovered_after"] or 999)["drop"]
            with open(os.path.join(a.out, f"trace_{bname}_{tag}_drop{pick}.txt"), "w", encoding="utf-8") as fh:
                fh.write("\n".join(trace([main_rows[pick - 2], main_rows[pick - 1]] + forks[pick], stream)) + "\n")
    with open(os.path.join(a.out, "summary.json"), "w", encoding="utf-8") as fh:
        json.dump(summary, fh, default=str, indent=1)
    print(f"total {time.time() - t0:.1f} s; outputs in {a.out}")


if __name__ == "__main__":
    main()
