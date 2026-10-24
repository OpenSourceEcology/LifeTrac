#!/usr/bin/env python3
"""Independent re-score of the 2026-10-24 FHSS round from the six harness archives.

Reads only the archives (params.txt, tx_daemon.log, rx_daemon.log, deploy_md5.txt,
tractor_ocd_reset.log). Written from scratch; does not import any repo module.
"""
import datetime as dt
import os
import re
import statistics
import sys

EV = sys.argv[1]
LEGS = [("2c_f1", "095315"), ("2b_f1", "100249"), ("2c_f2", "101341"),
        ("2b_f2", "102247"), ("2c_f3", "103156"), ("2b_f3", "104056")]

TS = re.compile(r"^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d{3}) ")


def ts_of(line):
    m = TS.match(line)
    if not m:
        return None
    return dt.datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S,%f").replace(tzinfo=dt.timezone.utc).timestamp()


def lines(path):
    with open(path, encoding="utf-8-sig", errors="replace") as fh:
        return [l.rstrip("\r\n") for l in fh if l.strip()]


def params(path):
    out = {}
    for l in lines(path):
        k, s, v = l.lstrip("\ufeff").partition("=")
        if s:
            out[k.strip()] = v.strip()
    return out


def utc(t):
    return dt.datetime.fromtimestamp(t, dt.timezone.utc).strftime("%H:%M:%S.%f")[:-3]


for tag, stamp in LEGS:
    arch = os.path.join(EV, f"radio_monitor_20261024_{stamp}_d28bc2ca")
    p = params(os.path.join(arch, "params.txt"))
    toff = float(p["tractor_clock_offset_s"])
    boff = float(p["base_clock_offset_s"])
    tx = lines(os.path.join(arch, "tx_daemon.log"))
    rx = lines(os.path.join(arch, "rx_daemon.log"))

    # ---------------- tractor (tx_daemon.log)
    txdone = []          # (pc_utc, seq_u8, status)
    done_k = []          # K of "frame seq=N done ... K fragments ok"
    aborted = 0
    tr_cmd = []          # (pc_utc, opname, seq, rxn)
    tr_other_cmd = []
    faults_t = []
    shutdown_t = None
    stats_t = None
    for l in tx:
        t = ts_of(l)
        m = re.search(r"txdone_arrival: seq=(\d+) idx=(\d+) status=(\d+)", l)
        if m:
            txdone.append((t - toff, int(m.group(1)), int(m.group(3)), int(m.group(2))))
            continue
        m = re.search(r"frame seq=(\d+) done .*?: (\d+) fragments ok", l)
        if m:
            done_k.append(int(m.group(2)))
            continue
        if "ABORT" in l:
            aborted += 1
        m = re.search(r"LoRa cmd: (\S+) seq=(\d+).*\(rx#(\d+)\)", l)
        if m:
            if m.group(1) == "PROBE":
                tr_cmd.append((t - toff, int(m.group(2)), int(m.group(3))))
            else:
                tr_other_cmd.append(l)
            continue
        if "LoRa cmd:" in l:
            tr_other_cmd.append(l)
        if "FAULT" in l:
            raw = re.search(r"code=(0x[0-9A-Fa-f]+) sub=(0x[0-9A-Fa-f]+) raw=([0-9a-f]+)", l)
            up = None
            if raw:
                b = bytes.fromhex(raw.group(3))
                up = int.from_bytes(b[20:24], "little") / 1000.0
            faults_t.append((utc(t - toff), raw.group(1) if raw else "?", raw.group(2) if raw else "?", up))
        if "signal received; shutting down" in l:
            shutdown_t = t - toff
        if "image_tx_daemon: stats:" in l:
            stats_t = l
    # unwrap the u8 seq of the tractor's TX order
    abs_tx = []
    wrap = 0
    prev = None
    for (t, s, st, idx) in txdone:
        if prev is not None and s < prev and prev - s > 128:
            wrap += 1
        abs_tx.append((t, s, s + 256 * wrap, st, idx))
        prev = s
    sent_ok = [a for a in abs_tx if a[3] == 0]
    sent_bad = [a for a in abs_tx if a[3] != 0]

    # ---------------- base (rx_daemon.log)
    frags = []           # (pc_utc, seq)
    published = 0
    probe_tx = []        # (pc_utc, seq, txn)
    ops = {}             # opcode -> [attempts, ok]
    rf = []              # (n, rssi_min, rssi_med, rssi_max, snr_min, snr_med, snr_max)
    faults_b = []
    shutdown_b = None
    for l in rx:
        t = ts_of(l)
        m = re.search(r"frag_arrival: seq=(\d+) idx=(\d+) total=(\d+)", l)
        if m:
            frags.append((t - boff, int(m.group(1))))
            continue
        if re.search(r"published frame_id=\d+ seq=\d+", l):
            published += 1
            continue
        m = re.search(r"PROBE TX seq=(\d+) .*\(tx#(\d+)\)", l)
        if m:
            probe_tx.append((t - boff, int(m.group(1)), int(m.group(2))))
            continue
        m = re.search(r"command TX opcode=0x([0-9a-fA-F]+) copy=\d+/\d+ (OK|FAILED)", l)
        if m:
            o = ops.setdefault(m.group(1).lower(), [0, 0])
            o[0] += 1
            o[1] += m.group(2) == "OK"
            continue
        m = re.search(r"rx_rf: n_rssi=(\d+) n_snr=(\d+) rssi\[min/med/max\]=(-?\d+)/(-?\d+)/(-?\d+) "
                      r"snr\[min/med/max\]=(-?[\d.]+)/(-?[\d.]+)/(-?[\d.]+)", l)
        if m:
            rf.append(tuple(float(x) for x in m.groups()))
            continue
        if "FAULT" in l:
            faults_b.append(l[:160])
        if "signal received; shutting down" in l:
            shutdown_b = t - boff

    # map each base frag to the tractor TX (same u8 seq, nearest PC time)
    mapped = []
    for (t, s) in frags:
        cands = [a for a in abs_tx if a[1] == s]
        best = min(cands, key=lambda a: abs(a[0] - t)) if cands else None
        mapped.append((t, s, best[2] if best else None, (t - best[0]) if best else None))
    abs_rx = sorted({m[2] for m in mapped if m[2] is not None})
    first_abs = abs_rx[0] if abs_rx else None
    last_abs = abs_rx[-1] if abs_rx else None
    span = (last_abs - first_abs + 1) if abs_rx else 0
    lost_after_lock = span - len(abs_rx) if abs_rx else None
    lat = [m[3] for m in mapped if m[3] is not None]

    # seq-gap from the base log alone, u8 with time-based wrap check
    # (independent of the tractor log): count missing between consecutive base seqs
    gaps_u8 = 0
    for (a, b) in zip(frags, frags[1:]):
        d = (b[1] - a[1]) % 256
        # elapsed frames at ~0.5 s per frame
        el = (b[0] - a[0]) / 0.5
        wraps = max(0, round((el - d) / 256))
        gaps_u8 += d - 1 + 256 * wraps

    # probes
    bseq = [x[1] for x in probe_tx]
    tseq = [x[1] for x in tr_cmd]
    delivered = sorted(set(tseq) & set(bseq))
    missing = [s for s in bseq if s not in set(tseq)]
    stray = sorted(set(tseq) - set(bseq))
    txn_ok = [x[2] for x in probe_tx] == list(range(1, len(probe_tx) + 1))
    rxn_ok = [x[2] for x in tr_cmd] == list(range(1, len(tr_cmd) + 1))
    # base PROBE->tractor log latency
    tmap = {x[1]: x[0] for x in tr_cmd}
    plat = [tmap[s] - t for (t, s, _) in probe_tx if s in tmap]

    # deploy md5
    dm = lines(os.path.join(arch, "deploy_md5.txt"))
    dstat = {}
    for l in dm:
        f = l.lstrip("\ufeff").split()
        dstat.setdefault((f[0], f[1]), 0)
        dstat[(f[0], f[1])] += 1
        if f[1] == "ok":
            pcm = re.search(r"pc=(\S+)", l).group(1)
            bm = re.search(r"board=(\S+)", l).group(1)
            if pcm != bm:
                dstat[("MD5DIFF", f[0])] = dstat.get(("MD5DIFF", f[0]), 0) + 1

    ocd = open(os.path.join(arch, "tractor_ocd_reset.log"), encoding="utf-8-sig").read()

    rssi_meds = [r[3] for r in rf]
    snr_meds = [r[6] for r in rf]
    print(f"=== leg {tag}  archive radio_monitor_20261024_{stamp}_d28bc2ca")
    print(f"  params: cam_extra_env={p['cam_extra_env']!r} reg_profile={p['reg_profile']} reactive_fire={p['reactive_fire']} "
          f"probe_echo={p['probe_echo']} tx_batch={p['tx_batch']} force_frf_hz={p['force_frf_hz']} duration_s={p['duration_s']} synth_fps={p['synth_fps']}")
    print(f"  params: log_window={p.get('log_window')} log_window_detail={p.get('log_window_detail')} stop_drain_s={p.get('stop_drain_s')} "
          f"feed_stop={p.get('feed_stop')!r}")
    print(f"  params: tractor_clock_offset_s={p['tractor_clock_offset_s']} (rtt {p['tractor_clock_rtt_s']}) "
          f"base_clock_offset_s={p['base_clock_offset_s']} (rtt {p['base_clock_rtt_s']})")
    print(f"  params: tractor_l072_reset={p['tractor_l072_reset']} ({p['tractor_l072_reset_reason']}) tag={p['tractor_ocd_tag']} "
          f"base_l072_reset={p['base_l072_reset']}")
    print(f"  params: tractor_deploy_md5={p['tractor_deploy_md5']!r} base_deploy_md5={p['base_deploy_md5']!r}")
    print(f"  deploy_md5.txt: {dstat}")
    print(f"  ocd log: banner={'Open On-Chip Debugger' in ocd} OCD_EXIT={re.search(r'OCD_EXIT=(\d+)', ocd).group(1)} "
          f"tag_match={p['tractor_ocd_tag'] in ocd} PF4 low/high={'GPIOF_IDR.PF_4 (low)  = 0' in ocd}/{'GPIOF_IDR.PF_4 (high) = 1' in ocd}")
    print(f"  tractor: txdone lines {len(abs_tx)} status=0 {len(sent_ok)} status!=0 {len(sent_bad)} idx set {sorted({a[4] for a in abs_tx})}; "
          f"'frame done' lines {len(done_k)} K dist {dict((k, done_k.count(k)) for k in sorted(set(done_k)))}; ABORT {aborted}; wraps {wrap}")
    if abs_tx:
        print(f"  tractor TX first {utc(abs_tx[0][0])}Z last {utc(abs_tx[-1][0])}Z (PC UTC via offset); shutdown {utc(shutdown_t)}Z")
    print(f"  tractor last stats: {stats_t[stats_t.find('stats:'):] if stats_t else None}")
    print(f"  tractor FAULTs: {faults_t}")
    print(f"  base: frag_arrival {len(frags)} published {published}; first {utc(frags[0][0]) if frags else None}Z seq {frags[0][1] if frags else None} "
          f"-> tractor abs frame #{first_abs} ; last abs #{last_abs}; base shutdown {utc(shutdown_b)}Z")
    print(f"  base frag->tractor TX latency (PC clocks): min {min(lat):.3f} med {statistics.median(lat):.3f} max {max(lat):.3f} s" if lat else "  no latency")
    print(f"  seq-gap after lock (tractor-mapped): {lost_after_lock}/{span} = {100.0*lost_after_lock/span:.2f} %; "
          f"base-only u8 gap sum {gaps_u8}; whole-log loss {len(sent_ok)-len(abs_rx)}/{len(sent_ok)} = {100.0*(len(sent_ok)-len(abs_rx))/len(sent_ok):.2f} %")
    unheard_head = first_abs if first_abs is not None else None
    print(f"  frames before first heard: {first_abs} (abs index, 0-based from the tractor's first TX); heard after lock {len(abs_rx)}")
    print(f"  probes: base PROBE TX {len(probe_tx)} (tx# contiguous {txn_ok}) seq {bseq[0] if bseq else None}..{bseq[-1] if bseq else None}; "
          f"tractor PROBE lines {len(tr_cmd)} distinct {len(set(tseq))} (rx# contiguous {rxn_ok}); delivered {len(delivered)}/{len(bseq)}; "
          f"missing {missing}; stray {stray}; tractor other cmds {len(tr_other_cmd)}")
    if plat:
        print(f"  PROBE base-send -> tractor-log latency (PC clocks): min {min(plat):.3f} med {statistics.median(plat):.3f} max {max(plat):.3f} s")
    print(f"  base command TX opcodes: {ops}")
    if rf:
        print(f"  rx_rf: {len(rf)} windows; RSSI median-of-medians {statistics.median(rssi_meds)} dBm (min med {min(rssi_meds)}, "
              f"min {min(r[2] for r in rf)}, max {max(r[4] for r in rf)}); SNR median-of-medians {statistics.median(snr_meds)} dB "
              f"(min med {min(snr_meds)}, min {min(r[5] for r in rf)}, max {max(r[7] for r in rf)}); frames {sum(r[0] for r in rf):.0f}")
    print(f"  base FAULTs: {faults_b}")
    # missing probes: context (time of the miss)
    for s in missing:
        t = [x[0] for x in probe_tx if x[1] == s][0]
        print(f"   missing PROBE seq {s} sent {utc(t)}Z")
    # silent stretches on the base side (PC times), > 3 s between frags, plus head/tail
    if frags:
        st = [(frags[0][0] - (abs_tx[0][0] if abs_tx else frags[0][0]), "head")]
        g = [(b[0] - a[0], utc(a[0])) for a, b in zip(frags, frags[1:]) if b[0] - a[0] > 3]
        tail = (abs_tx[-1][0] - frags[-1][0]) if abs_tx else 0
        print(f"  base silent: head {st[0][0]:.1f} s after tractor first TX; gaps>3s {[(round(x, 1), y) for x, y in g]}; tail {tail:.1f} s before tractor last TX")
    print()
