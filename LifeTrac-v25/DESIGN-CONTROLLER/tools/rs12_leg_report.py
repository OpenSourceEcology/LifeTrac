"""rs12_leg_report.py — one-command leg analysis for the RS-12 campaign.

Reports, for a radio-monitor archive: raw loss, the per-index loss
histogram with the penultimate share, the corrupt-capture index
distribution (read out of the crc_dump payload headers), and — when given
bracketing stats-probe snapshots — the radio-counter deltas and the
firmware-drop count (drx_ok - host URCs).

Usage:
    py -3 rs12_leg_report.py <archive_dir> [--pre stats_pre.txt --post stats_post.txt]
"""

from __future__ import annotations

import argparse
import collections
import pathlib
import re


def parse_stats(path: pathlib.Path) -> dict[str, int]:
    out = {}
    for m in re.finditer(r"(\w+)=(\d+)", path.read_text(encoding="utf-8",
                                                        errors="replace")):
        out[m.group(1)] = int(m.group(2))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("archive", type=pathlib.Path)
    ap.add_argument("--pre", type=pathlib.Path)
    ap.add_argument("--post", type=pathlib.Path)
    args = ap.parse_args()

    rx = (args.archive / "rx_daemon.log").read_text(encoding="utf-8",
                                                    errors="replace")
    tx = (args.archive / "tx_daemon.log").read_text(encoding="utf-8",
                                                    errors="replace")
    params = (args.archive / "params.txt").read_text(encoding="utf-8",
                                                     errors="replace")

    for key in ("train_gap_ms", "tx_pipeline_depth", "synth_budget_b",
                "force_frf_hz", "git_sha"):
        m = re.search(rf"{key}=(\S+)", params)
        if m:
            print(f"{key}={m.group(1)}", end="  ")
    print()

    sent = int(re.findall(r"frags_ok=(\d+)", tx)[-1])
    rcvd = int(re.findall(r"rx_frames=(\d+)", rx)[-1])
    crc = rx.count("crc_dump")
    timeouts = int(re.findall(r"reassembler_timeouts=(\d+)", rx)[-1])
    published = int(re.findall(r"frames_published=(\d+)", rx)[-1])
    print(f"loss {sent - rcvd}/{sent} = {100 * (sent - rcvd) / sent:.1f}%   "
          f"crc_dumps={crc}  timeouts={timeouts}  published={published}")

    # per-index loss histogram (attribution instrument)
    idx: collections.Counter = collections.Counter()
    for m in re.finditer(r"lost_frag_idx: n=\d+.*?\| top ([0-9: ]+)", rx):
        for pair in m.group(1).split():
            i, c = pair.split(":")
            idx[int(i)] += int(c)
    tot = sum(idx.values())
    if tot:
        # train length from the most common `total` byte in dumps, fallback 13
        totals = collections.Counter()
        for m in re.finditer(r"dump=([0-9a-f]{24,})", rx):
            d = bytes.fromhex(m.group(1)[:24])
            if d[8] == 0xFE:
                totals[d[11] + 1] += 1
        tlen = totals.most_common(1)[0][0] if totals else 13
        pen = tlen - 2
        print(f"train length {tlen}; attributed {tot}; "
              f"penultimate idx {pen} = {idx.get(pen, 0)} "
              f"({100 * idx.get(pen, 0) / tot:.0f}%)")
        print("  " + " ".join(f"{i}:{idx.get(i, 0)}" for i in range(tlen)))

        # corrupt-capture indices
        cidx: collections.Counter = collections.Counter()
        for m in re.finditer(r"dump=([0-9a-f]{24,})", rx):
            d = bytes.fromhex(m.group(1)[:24])
            if d[8] == 0xFE and d[11] + 1 == tlen:
                cidx[d[10]] += 1
        ctot = sum(cidx.values())
        if ctot:
            print(f"corrupt-capture idx (readable {ctot}): penultimate "
                  f"{cidx.get(pen, 0)} ({100 * cidx.get(pen, 0) / ctot:.0f}% "
                  f"vs uniform {100 / tlen:.0f}%)")

    if args.pre and args.post:
        pre = parse_stats(args.pre)
        post = parse_stats(args.post)
        d = {k: post[k] - pre[k] for k in
             ("radio_dio0", "radio_rx_ok", "radio_crc_err", "radio_tx_ok")
             if k in pre and k in post}
        print(f"radio deltas: {d}")
        fw_drop = d.get("radio_rx_ok", 0) - rcvd
        print(f"FIRMWARE DROP (drx_ok - host URCs): "
              f"{d.get('radio_rx_ok', 0)} - {rcvd} = {fw_drop}")
        resid = d.get("radio_dio0", 0) - sum(
            d.get(k, 0) for k in ("radio_rx_ok", "radio_crc_err",
                                  "radio_tx_ok"))
        print(f"identity residue: {resid}")
        print(f"crc closure: dcrc_err={d.get('radio_crc_err', 0)} "
              f"vs crc_dumps={crc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
