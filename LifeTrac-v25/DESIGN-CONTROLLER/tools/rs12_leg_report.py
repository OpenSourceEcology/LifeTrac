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
        # Train length comes from the TX log's healthy "K fragments ok"
        # lines, and the penultimate metric is scoped to the LONG-train
        # population (>= MIN_LONG fragments).
        #
        # 2026-08-24, two bugs found in one leg:
        #  (a) this used to read the `total` byte out of CORRUPT capture
        #      headers — the one population whose bytes are unreliable —
        #      with most_common(1) and no sanity bound. A control leg
        #      with a single readable dump reported train length 208 off
        #      that one garbage byte, moving "penultimate" to idx 206 and
        #      printing 0 % while the real lock sat at 35 % on idx 11: a
        #      false NEGATIVE on the campaign's headline metric.
        #  (b) a plain modal over ALL trains is also wrong — these legs
        #      carry a MIXTURE (e.g. 143x1, 45x2, 56x12, 115x13), so the
        #      overall mode is 1 and "penultimate" collapses to idx -1.
        #      Short trains have no penultimate to lock, so the metric is
        #      only meaningful over long trains.
        # The mixture is printed so a reader can never mistake a
        # mixed-length leg for a uniform one.
        MIN_LONG = 3
        tx_lens = collections.Counter(
            int(n) for n in re.findall(r"(\d+) fragments ok", tx))
        long_lens = collections.Counter(
            {k: v for k, v in tx_lens.items() if k >= MIN_LONG})
        if long_lens:
            tlen = long_lens.most_common(1)[0][0]
            tlen_src = (f"tx log, modal of {sum(long_lens.values())} long "
                        f"trains; mixture " +
                        " ".join(f"{k}x{v}" for k, v in
                                 sorted(tx_lens.items())))
        else:
            tlen = 13
            tlen_src = "DEFAULT 13 — no long trains in tx log, treat with care"
        pen = tlen - 2
        print(f"train length {tlen} [{tlen_src}]")
        print(f"attributed {tot}; penultimate idx {pen} = {idx.get(pen, 0)} "
              f"({100 * idx.get(pen, 0) / tot:.0f}%, uniform "
              f"{100 / tlen:.0f}%)")

        # MIXED-LENGTH AMBIGUITY (review catch, PR #112). The rx daemon's
        # lost_frag_idx instrument reports an index with no train length,
        # so this histogram aggregates every length present. When more
        # than one long length occurs, a single index means different
        # things per population: at lengths 12 and 13, idx 11 is the
        # FINAL fragment of the 12s and the PENULTIMATE of the 13s. The
        # share printed above is therefore mixture-weighted, not a clean
        # per-population figure. Say so rather than letting the label
        # imply a purity the instrument cannot deliver.
        other_long = sorted(k for k in long_lens if k != tlen)
        if other_long:
            print(f"  !! MIXED LENGTHS {sorted(long_lens)} — idx {pen} is "
                  f"penultimate for len {tlen}, but also "
                  + ", ".join(
                      f"{'final' if pen == k - 1 else f'idx {pen}'} of "
                      f"len {k}" for k in other_long)
                  + f"; each length's own penultimate: "
                  + ", ".join(f"len {k}->idx {k - 2}={idx.get(k - 2, 0)}"
                              for k in sorted(long_lens))
                  + ". Shares are mixture-weighted; separating them needs "
                    "lost_frag_idx to carry the train length.")
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
        # RS-12 (2026-09-12): URC-path loss counters, present only on the
        # instrumented build; absent keys print as n/a so old brackets
        # still analyse.
        urc = {k: (post[k] - pre[k]) if (k in pre and k in post) else "n/a"
               for k in ("rx_urc_lost", "rx_pretx_drained")}
        print(f"URC-path: rx_urc_lost={urc['rx_urc_lost']} "
              f"rx_pretx_drained={urc['rx_pretx_drained']}")
        # RS-12.10 (2026-09-12): FIFO-skip detector (the coalescing the edge
        # counter cannot see) and the TX deaf window. max fields are NOT
        # deltas — the bracket max is the leg max only when the counters
        # were reset at launch; sum is a true delta.
        keys = ("rx_fifo_skip", "tx_deaf_sum_us")
        maxk = ("tx_deaf_max_us", "tx_done_to_rearm_max_us")
        if all(k in pre and k in post for k in keys + maxk):
            tx_n = max(d.get("radio_tx_ok", 0), 1)
            deaf_sum = post["tx_deaf_sum_us"] - pre["tx_deaf_sum_us"]
            print(f"RS-12.10: rx_fifo_skip={post['rx_fifo_skip'] - pre['rx_fifo_skip']} "
                  f"tx_deaf_sum={deaf_sum} us over {tx_n} TX "
                  f"(mean {deaf_sum / tx_n / 1000.0:.1f} ms) "
                  f"tx_deaf_max={post['tx_deaf_max_us'] / 1000.0:.1f} ms "
                  f"tx_done_to_rearm_max={post['tx_done_to_rearm_max_us']} us "
                  f"(max fields: post-bracket values)")
        else:
            print("RS-12.10: n/a (counters absent in a bracket)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
