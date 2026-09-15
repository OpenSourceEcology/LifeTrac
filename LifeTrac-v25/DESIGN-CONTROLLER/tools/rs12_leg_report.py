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

    # PR #125 review round 6 (2026-09-15): `frags_ok=` is the TX daemon's LAST
    # periodic stats line and can be stale independently of the RX stats
    # thread -- in leg U it read 652 while the per-frame completion events
    # ("frame seq=N done ...: K fragments ok", one per train, logged as each
    # train finishes) summed to 674. Those events are the fragments the L072
    # reported on air (TX_DONE OK); aborted fragments never appear in them
    # and were never on air, so they are the right air-loss denominator.
    # Prefer the events; fall back to the counter only for a log without them.
    sent_counter = int(re.findall(r"frags_ok=(\d+)", tx)[-1])
    sent_events = sum(int(n) for n in re.findall(r"(\d+) fragments ok", tx))
    sent = sent_events if sent_events else sent_counter
    if sent_events and sent_events != sent_counter:
        print(f"  (tx frags_ok counter {sent_counter} is stale; using "
              f"{sent_events} fragments from per-frame TX events)")
    rcvd = int(re.findall(r"rx_frames=(\d+)", rx)[-1])
    crc = rx.count("crc_dump")
    timeouts = int(re.findall(r"reassembler_timeouts=(\d+)", rx)[-1])
    published = int(re.findall(r"frames_published=(\d+)", rx)[-1])

    # 2026-09-15 (leg U): EVERY number on the loss line comes from the LAST
    # "stats:" line in the rx log. If the daemon's stats thread dies mid-leg
    # that line is whatever was true when it died, and this report then
    # describes the first seconds as if they were the whole leg — silently,
    # and in the direction of a false alarm. Leg U crashed that thread one
    # frame in and this printed "loss 651/652 = 99.8% published=1" for a leg
    # that actually published 537 frames with zero lock losses. Cross-check
    # against events logged once per frame/fragment, which cannot go stale.
    pub_log = rx.count("published frame_id")
    frag_log = rx.count("frag_arrival")          # needs -LogFragArrivals 1
    stats_dead = "Exception in thread image-rx-stats" in rx
    counters_stale = stats_dead or (pub_log and published < pub_log * 0.9)
    if counters_stale:
        why = ("the stats thread CRASHED (traceback in rx_daemon.log)"
               if stats_dead else
               "the counter line disagrees with the per-frame log events")
        print(f"  !! STALE COUNTERS: {why} — the loss line below describes "
              f"only the window before it stopped updating. Trust these:")
        if frag_log:
            print(f"     log-derived: published={pub_log}  "
                  f"fragments_arrived={frag_log}  "
                  f"air_loss={100 * (sent - frag_log) / sent:.1f}% "
                  f"({sent} sent -> {frag_log} decoded)")
        else:
            print(f"     log-derived: published={pub_log}  "
                  f"(re-run with -LogFragArrivals 1 for fragment-level loss)")
    print(f"loss {sent - rcvd}/{sent} = {100 * (sent - rcvd) / sent:.1f}%   "
          f"crc_dumps={crc}  timeouts={timeouts}  published={published}"
          + ("   [STALE — see above]" if counters_stale else ""))

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
            tlen_src = "DEFAULT 13 -- no long trains in tx log, treat with care"
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
            print(f"  !! MIXED LENGTHS {sorted(long_lens)} -- idx {pen} is "
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
        # The probe dump lists only NON-ZERO counters, so a zero reads as
        # absent: treat any RS-12.10 key present in the post bracket as
        # proof of the instrumented build and default the rest to 0.
        keys = ("rx_fifo_skip", "tx_deaf_sum_us")
        maxk = ("tx_deaf_max_us", "tx_done_to_rearm_max_us")
        if any(k in post for k in keys + maxk):
            pre = {**{k: 0 for k in keys + maxk}, **pre}
            post = {**{k: 0 for k in keys + maxk}, **post}
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
        # RS-12.15 v2 (2026-09-14): FHSS clock-authority counters. Deltas of
        # the decision histogram and the demotion outcome; streak_max and
        # first_anchor are post-bracket values (first_anchor is cumulative
        # since firmware reset -- report the delta AND the post value).
        rs15 = ("fhss_dec_aligned", "fhss_dec_snapped", "fhss_dec_rej_not_init",
                "fhss_dec_rej_bad_hop", "fhss_dec_rej_epoch_drift",
                "fhss_dec_rej_locked_out", "clk_demotion_reset",
                "clk_demotion_kept", "tx_first_anchor", "tx_stream_streak_max")
        if any(k in post for k in rs15):
            pre15 = {**{k: 0 for k in rs15}, **pre}
            post15 = {**{k: 0 for k in rs15}, **post}
            d15 = {k: post15[k] - pre15[k] for k in rs15}
            print(f"RS-12.15: consider_remote deltas aligned={d15['fhss_dec_aligned']} "
                  f"snapped={d15['fhss_dec_snapped']} locked_out={d15['fhss_dec_rej_locked_out']} "
                  f"epoch_drift={d15['fhss_dec_rej_epoch_drift']} bad_hop={d15['fhss_dec_rej_bad_hop']} "
                  f"not_init={d15['fhss_dec_rej_not_init']} | demotion reset={d15['clk_demotion_reset']} "
                  f"kept={d15['clk_demotion_kept']} | tx_first_anchor delta={d15['tx_first_anchor']} "
                  f"(post {post15['tx_first_anchor']}) | streak_max post={post15['tx_stream_streak_max']}")
        else:
            print("RS-12.15: n/a (counters absent in a bracket)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
