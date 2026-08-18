"""rs12_txrx_join.py — join TX_DONE timing with RX arrival gaps per train.

For every train that arrives at the base missing exactly its penultimate
fragment, pull the TRACTOR's txdone_arrival records for that same train
(joined on seq — no cross-board clock arithmetic) and report:

  * whether idx N-2 got a TX_DONE at all, its status, and its toa_us
    (full-length ~99904 us for 255 B, or truncated)
  * the TX-side wall gaps TX_DONE(N-3)->(N-2)->(N-1):
      ~117 ms = host-paced normal
      ~100 ms = fired back-to-back (ToA-limited, no pacing gap)
      <100 ms = overlapped / early TX_DONE — the self-collision signature

Clean trains give the control distribution.

Usage:
    py -3 rs12_txrx_join.py <archive_dir>
"""

from __future__ import annotations

import argparse
import pathlib
import re
import statistics

RX_LINE = re.compile(
    r"frag_arrival: seq=(\d+) idx=(\d+) total=(\d+) fw_us=(\d+)")
TX_LINE = re.compile(
    r"(\d{2}):(\d{2}):(\d{2}),(\d{3}) INFO image_tx_daemon: "
    r"txdone_arrival: seq=(\d+) idx=(\d+) status=(\d+) toa_us=(\d+)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("archive", type=pathlib.Path)
    args = ap.parse_args()

    rx_txt = (args.archive / "rx_daemon.log").read_text(encoding="utf-8",
                                                        errors="replace")
    tx_txt = (args.archive / "tx_daemon.log").read_text(encoding="utf-8",
                                                        errors="replace")

    # RX: trains by seq (in arrival order), noting which indices made it
    rx_trains: dict[int, dict] = {}
    order: list[int] = []
    for m in RX_LINE.finditer(rx_txt):
        seq, idx, total, fw = map(int, m.groups())
        t = rx_trains.setdefault(seq, {"total": total, "have": set()})
        t["have"].add(idx)
        if seq not in order:
            order.append(seq)

    # TX: per (seq, idx) wall-ms + toa + status
    tx: dict[tuple[int, int], dict] = {}
    for m in TX_LINE.finditer(tx_txt):
        h, mi, s, ms, seq, idx, status, toa = map(int, m.groups())
        tx[(seq, idx)] = {
            "wall": (h * 3600 + mi * 60 + s) * 1000 + ms,
            "status": status, "toa": toa}
    print(f"RX trains: {len(rx_trains)}   TX txdone records: {len(tx)}")
    if not tx:
        print("no txdone_arrival lines — TX instrument not active?")
        return 1

    def tx_gap(seq: int, a: int, b: int) -> float | None:
        if (seq, a) in tx and (seq, b) in tx:
            return tx[(seq, b)]["wall"] - tx[(seq, a)]["wall"]
        return None

    drop_g_10_11, drop_g_11_12 = [], []
    clean_g_10_11, clean_g_11_12 = [], []
    pen_toa_drop, pen_toa_clean = [], []
    missing_txdone = 0
    n_drop = 0

    for seq, t in rx_trains.items():
        total = t["total"]
        pen = total - 2
        if not ((pen - 1) in t["have"] and (pen + 1) in t["have"]):
            continue
        g1 = tx_gap(seq, pen - 1, pen)
        g2 = tx_gap(seq, pen, pen + 1)
        if pen not in t["have"]:
            n_drop += 1
            if (seq, pen) not in tx:
                missing_txdone += 1
                continue
            pen_toa_drop.append(tx[(seq, pen)]["toa"])
            if g1 is not None:
                drop_g_10_11.append(g1)
            if g2 is not None:
                drop_g_11_12.append(g2)
        else:
            if (seq, pen) in tx:
                pen_toa_clean.append(tx[(seq, pen)]["toa"])
            if g1 is not None:
                clean_g_10_11.append(g1)
            if g2 is not None:
                clean_g_11_12.append(g2)

    print(f"\npenultimate-drop trains (RX view): {n_drop}   "
          f"of which missing a TX_DONE for the lost idx: {missing_txdone}")

    def show(label, drop, clean):
        if drop:
            print(f"  {label}: DROP med={statistics.median(drop):6.1f} ms "
                  f"min={min(drop):6.1f} max={max(drop):6.1f} (n={len(drop)})")
        if clean:
            print(f"  {label:{len(label)}s}  CLEAN med="
                  f"{statistics.median(clean):6.1f} ms (n={len(clean)})")

    print("\nTX-side wall gaps around the penultimate:")
    show("TXDONE(N-3)->(N-2)", drop_g_10_11, clean_g_10_11)
    show("TXDONE(N-2)->(N-1)", drop_g_11_12, clean_g_11_12)

    print("\npenultimate toa_us as reported by TX_DONE:")
    if pen_toa_drop:
        print(f"  drop trains : med={statistics.median(pen_toa_drop):.0f} "
              f"min={min(pen_toa_drop)} max={max(pen_toa_drop)}")
    if pen_toa_clean:
        print(f"  clean trains: med={statistics.median(pen_toa_clean):.0f}")
    print("  (255 B full-length ToA = 99904 us)")

    # per-drop-train detail for the record
    print("\nper-drop-train detail:")
    for seq, t in rx_trains.items():
        total = t["total"]
        pen = total - 2
        if pen in t["have"] or not ((pen - 1) in t["have"]
                                    and (pen + 1) in t["have"]):
            continue
        g1 = tx_gap(seq, pen - 1, pen)
        g2 = tx_gap(seq, pen, pen + 1)
        rec = tx.get((seq, pen))
        print(f"  seq={seq:3d} gap({pen-1}->{pen})="
              f"{g1 if g1 is not None else '?':>6} ms  "
              f"gap({pen}->{pen+1})={g2 if g2 is not None else '?':>6} ms  "
              f"toa={rec['toa'] if rec else '(no TX_DONE)'} "
              f"status={rec['status'] if rec else '-'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
