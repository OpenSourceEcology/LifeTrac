"""interferer_replay.py — does RS-4.1 XOR parity actually rescue the losses
we measured?

Drives the REAL shipping code, not a model of it:
  TX side  base_station/lora_proto.py::add_parity_fragments
  RX side  base_station/image_pipeline/reassemble.py::FragmentReassembler

against the loss process measured in RS-11.6 item 1, and reports frame
delivery with and without parity plus the airtime each option costs.

THE QUESTION THIS EXISTS TO ANSWER. XOR parity repairs exactly ONE loss per
group. So it is not the loss RATE that decides whether parity helps, it is
whether losses ARRIVE ALONE. RS-11.6 item 1 found two populations with very
different arrival statistics -- a hard-periodic interferer at 7.085 s and a
diffuse bulk process -- so the mix matters, and the periodic component could
in principle alias against the train cadence and drop repeatedly into the
same parity group, which would make parity worthless. That is the failure
mode being tested.

MEASURED INPUTS (leg-1 archive radio_monitor_20260816_122735_9c891670):
  fragment spacing        117 ms      (air_gap dt_med=117032us)
  train boundary          213.9 ms    (RS-11.1 measured)
  fragments per train     13          (3000 B budget / ~247 B median)
  total fragment loss     5.9 %       (141 / 2405)
  interferer period       7.0853 s    (RS-11.6 item 1 grid fit)
  interferer share        11 / 86 observed corrupt captures = 12.8 %

The interferer share is the least certain input -- corrupt CAPTURES are not
the same population as total losses, since a fragment can be lost without
producing a crc_dump at all. So the share is swept rather than asserted.
"""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import random
import sys

_HERE = pathlib.Path(__file__).resolve().parent
_BS = _HERE.parent / "base_station"
if str(_BS) not in sys.path:
    sys.path.insert(0, str(_BS))

os.environ.setdefault("LIFETRAC_FLEET_KEY_HEX",
                      "0102030405060708090a0b0c0d0e0f10")

from image_pipeline.reassemble import (  # noqa: E402
    FRAGMENT_MAGIC,
    FragmentReassembler,
)
from lora_proto import add_parity_fragments  # noqa: E402

# ------------------------------------------------------------ measured inputs

FRAG_MS = 117.0
TRAIN_GAP_MS = 213.9
TRAIN_FRAGS = 13
BODY_LEN = 243
RUN_S = 300.0
INTERFERER_PERIOD_S = 7.0853
TOTAL_LOSS_RATE = 0.059
INTERFERER_SHARE = 11.0 / 86.0
# RS-11.4: 23-34% of trains time out at this loss rate. Midpoint 28.5%.
AFFECTED_TRAIN_FRAC = 0.285

# SF7/BW500 CR4/5, 255 B on air. Measured and recorded in RS-11.0.
TOA_255_US = 99904.0


def frag_v1(seq: int, idx: int, total: int, body: bytes) -> bytes:
    """A v1 (0xFE) image fragment, matching the wire format the daemons use."""
    return bytes([FRAGMENT_MAGIC, seq & 0xFF, idx & 0xFF,
                  (total - 1) & 0xFF]) + body


def lora_toa_us(sf: int, bw_hz: int, cr_den: int, pl: int,
                crc: int = 1, ih: int = 0, pre: int = 8, de: int = 0) -> float:
    ts = (2 ** sf) / bw_hz
    num = 8 * pl - 4 * sf + 28 + 16 * crc - 20 * ih
    den = 4 * (sf - 2 * de)
    n = 8 + max(math.ceil(num / den) * cr_den, 0)
    return ((pre + 4.25) * ts + n * ts) * 1e6


def build_timeline(n_frames: int, frags_per_train: int) -> list[tuple[int, int, float]]:
    """(frame_idx, frag_idx, t_seconds) for every fragment actually transmitted.

    Parity fragments occupy real slots and real airtime, so the caller passes
    the POST-parity count. That is the honest accounting: parity does not ride
    for free, it lengthens the train and adds its own exposure to loss.
    """
    out = []
    t = 0.0
    for f in range(n_frames):
        for i in range(frags_per_train):
            out.append((f, i, t))
            t += FRAG_MS / 1000.0
        t += TRAIN_GAP_MS / 1000.0
    return out


def interferer_hits(duration: float, phase: float) -> list[float]:
    k = 0
    out = []
    while True:
        t = phase + k * INTERFERER_PERIOD_S
        if t > duration:
            break
        out.append(t)
        k += 1
    return out


def simulate(group_len: int, interferer_share: float, seed: int,
             frames: int = 160, clustered: bool = True) -> dict:
    """One run. group_len=0 disables parity.

    Returns delivery statistics and the airtime multiplier parity costs.
    """
    rng = random.Random(seed)

    # Post-parity fragment count per train.
    if group_len > 0:
        n_parity = math.ceil(TRAIN_FRAGS / group_len)
    else:
        n_parity = 0
    frags_per_train = TRAIN_FRAGS + n_parity

    timeline = build_timeline(frames, frags_per_train)
    total_frags = len(timeline)
    duration = timeline[-1][2] if timeline else 0.0

    # --- how many losses of each kind
    n_loss_total = int(round(TOTAL_LOSS_RATE * total_frags))
    n_loss_int = int(round(n_loss_total * interferer_share))

    lost: set[tuple[int, int]] = set()

    # Interferer: losses land on the 7.085 s grid. Each hit takes whichever
    # fragment is in flight at that instant -- which is the whole point, since
    # it makes their placement CORRELATED in time rather than independent.
    #
    # FEASIBILITY (review catch, PR #99): a 7.085 s emitter produces only
    # ~duration/7.085 ticks per run (~61-65 here), so a requested share can be
    # PHYSICALLY IMPOSSIBLE -- share=1.0 asks for ~200 interferer losses when
    # only ~65 ticks exist. The first version of this code silently dropped
    # the unplaced losses, which cut TOTAL loss to ~a third of calibration in
    # the 60%/100% sweep rows and made parity look better for the wrong
    # reason. Now the shortfall goes back into the bulk pool so total loss
    # stays calibrated, and the ACHIEVED share is reported alongside the
    # requested one.
    hits = interferer_hits(duration, rng.uniform(0.0, INTERFERER_PERIOD_S))
    rng.shuffle(hits)
    placed = 0
    # Collision = the tick lands DURING a fragment's actual on-air time
    # [start, start + ToA), not merely near a start. The first revision
    # accepted the nearest start within +/-117 ms -- a 234 ms exposure
    # window for a 99.904 ms packet, overstating collision probability
    # ~2.3x (review catch, PR #99). With the honest window the tick-supply
    # feasibility ceiling TIGHTENS, strengthening the removing-the-emitter
    # bound.
    toa_s = TOA_255_US / 1e6
    starts = [e[2] for e in timeline]
    import bisect
    for ht in hits:
        if placed >= n_loss_int:
            break
        i = bisect.bisect_right(starts, ht) - 1
        if i >= 0 and ht < starts[i] + toa_s:
            key = (timeline[i][0], timeline[i][1])
            if key not in lost:
                lost.add(key)
                placed += 1

    n_loss_bulk = n_loss_total - placed

    # Bulk. THIS IS THE PART THAT DECIDES THE ANSWER, so it is calibrated
    # against measurement rather than assumed independent.
    #
    # An IID model at 5.9% over a 13-fragment train predicts
    # 1 - 0.941^13 = 55% of trains losing at least one fragment. RS-11.4
    # measured only 23-34% of trains timing out at that same loss rate, and
    # described the loss as a per-boundary EVENT of ~1.3-1.8 fragments. Both
    # say the same thing: losses CLUSTER. Fewer trains are hit, and the ones
    # that are hit lose several fragments.
    #
    # That distinction is the whole question for XOR parity, which repairs
    # exactly one loss per group. Two losses inside one group is a frame lost
    # no matter how much parity you paid for. So modelling bulk loss as IID
    # would flatter parity badly.
    #
    # Calibration: with mean loss L = TOTAL_LOSS_RATE * TRAIN_FRAGS per train
    # and an affected-train fraction A, events per train is
    # lam = -ln(1 - A) and fragments per event is L / lam.
    if clustered:
        lam = -math.log(1.0 - AFFECTED_TRAIN_FRAC)
        per_event = (TOTAL_LOSS_RATE * TRAIN_FRAGS) / lam
        remaining = n_loss_bulk
        guard = 0
        while remaining > 0 and guard < 10 ** 6:
            guard += 1
            frame = rng.randrange(frames)
            burst = max(1, int(round(rng.expovariate(1.0 / per_event))))
            burst = min(burst, frags_per_train)
            # Start far enough back that the whole burst fits inside the
            # train. Clamping instead (min(start+k, last)) would pile the
            # overflow onto the FINAL index, which is exactly the one
            # _try_parity_reconstruct refuses to rebuild -- a bias that
            # silently understates parity.
            start = rng.randrange(frags_per_train - burst + 1)
            for k in range(burst):
                if remaining <= 0:
                    break
                key = (frame, start + k)
                if key not in lost:
                    lost.add(key)
                    remaining -= 1
    else:
        guard = 0
        while len(lost) < n_loss_total and guard < 10 ** 6:
            guard += 1
            e = timeline[rng.randrange(len(timeline))]
            lost.add((e[0], e[1]))

    # --- replay through the REAL reassembler
    ras = FragmentReassembler()
    delivered = 0
    for f in range(frames):
        bodies = [bytes([(f + i) & 0xFF]) * BODY_LEN for i in range(TRAIN_FRAGS)]
        frags = [frag_v1(f & 0xFF, i, TRAIN_FRAGS, bodies[i])
                 for i in range(TRAIN_FRAGS)]
        if group_len > 0:
            frags = add_parity_fragments(frags, f & 0xFF, group_len)

        for slot, raw in enumerate(frags):
            if (f, slot) in lost:
                continue
            ras.feed(raw)

        # A frame is delivered if every data index is present, whether it
        # arrived or was reconstructed.
        partial = ras._partials.get(f & 0xFF)
        if partial is None:
            delivered += 1          # completed and retired
        elif partial.total > 0 and len(partial.parts) >= partial.total:
            delivered += 1
        ras._partials.pop(f & 0xFF, None)
        ras._completed_recent.clear()

    affected = len({f for (f, _) in lost})
    airtime_mult = frags_per_train / TRAIN_FRAGS
    return {
        "affected_train_pct": 100.0 * affected / frames,
        "requested_int": n_loss_int,
        "placed_int": placed,
        "achieved_share": placed / n_loss_total if n_loss_total else 0.0,
        "group_len": group_len,
        "frames": frames,
        "delivered": delivered,
        "delivery_pct": 100.0 * delivered / frames,
        "frags_per_train": frags_per_train,
        "airtime_mult": airtime_mult,
        "parity_recon": ras.stats.parity_reconstructions,
        "n_loss_total": n_loss_total,
        "n_loss_int": n_loss_int,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--trials", type=int, default=12)
    ap.add_argument("--frames", type=int, default=160)
    args = ap.parse_args()

    print("=" * 74)
    print("RS-4.1 XOR parity against the RS-11.6 measured loss process")
    print("=" * 74)
    print(f"train={TRAIN_FRAGS} frags  spacing={FRAG_MS}ms  "
          f"loss={TOTAL_LOSS_RATE * 100:.1f}%  "
          f"interferer period={INTERFERER_PERIOD_S}s")
    print(f"trials={args.trials}  frames/trial={args.frames}")

    for clustered in (False, True):
        model = ("CLUSTERED (calibrated to RS-11.4's 23-34% affected trains)"
                 if clustered else
                 "IID (independent per-fragment loss) -- SHOWN ONLY AS A FOIL")
        print(f"\n== loss model: {model} ==")
        print(f"{'parity':>8} {'frags':>6} {'airtime':>8} "
              f"{'delivery (mean +/- sd)':>24} {'trains hit':>11}")
        print("-" * 78)
        baseline = None
        for g in (0, 4, 8, 13):
            runs = [simulate(g, INTERFERER_SHARE, seed=1000 + t,
                             frames=args.frames, clustered=clustered)
                    for t in range(args.trials)]
            vals = [r["delivery_pct"] for r in runs]
            d = sum(vals) / len(vals)
            sd = math.sqrt(sum((v - d) ** 2 for v in vals) / max(1, len(vals) - 1))
            hit = sum(r["affected_train_pct"] for r in runs) / len(runs)
            r0 = runs[0]
            if baseline is None:
                baseline = d
            label = "off" if g == 0 else str(g)
            delta = "" if g == 0 else f"  ({d - baseline:+.1f} pts)"
            print(f"{label:>8} {r0['frags_per_train']:>6} "
                  f"{r0['airtime_mult']:>7.2f}x "
                  f"{d:>15.1f}% +/- {sd:4.1f} "
                  f"{hit:>10.1f}%{delta}")

    # --- sensitivity to the least certain input.
    # "requested" is the share asked for; "achieved" is what the 7.085 s tick
    # supply could actually deliver with total loss held at calibration
    # (shortfall returns to the bulk pool). Rows where the two diverge are
    # physically infeasible AT THIS TICK CADENCE and are shown to bound the
    # trend, not as attainable operating points.
    print(f"\n{'share req->achieved':>21}  " +
          "  ".join(f"G={g:<5}" for g in (0, 4, 8, 13)))
    print("-" * 78)
    for share in (0.0, 0.128, 0.30, 0.60, 1.0):
        cells = []
        ach = []
        for g in (0, 4, 8, 13):
            runs = [simulate(g, share, seed=2000 + t, frames=args.frames)
                    for t in range(args.trials)]
            cells.append(sum(r["delivery_pct"] for r in runs) / len(runs))
            ach.append(sum(r["achieved_share"] for r in runs) / len(runs))
        a = sum(ach) / len(ach)
        flag = "" if abs(a - share) < 0.02 else "  (infeasible as requested)"
        print(f"{share * 100:>9.0f}% -> {a * 100:>5.1f}%  " +
              "  ".join(f"{c:>6.1f}%" for c in cells) + flag)

    # --- airtime cost of the alternative
    print("\n-- airtime cost comparison --")
    t45 = lora_toa_us(7, 500000, 5, 255)
    t48 = lora_toa_us(7, 500000, 8, 255)
    print(f"   CR4/5 255B ToA = {t45 / 1000:.1f} ms")
    print(f"   CR4/8 255B ToA = {t48 / 1000:.1f} ms  "
          f"({100 * (t48 / t45 - 1):+.0f}% airtime)")
    for g in (4, 8, 13):
        mult = (TRAIN_FRAGS + math.ceil(TRAIN_FRAGS / g)) / TRAIN_FRAGS
        print(f"   parity G={g:<2} airtime {100 * (mult - 1):+.0f}%  "
              f"(train {TRAIN_FRAGS} -> "
              f"{TRAIN_FRAGS + math.ceil(TRAIN_FRAGS / g)} fragments, "
              f"latency {(TRAIN_FRAGS + math.ceil(TRAIN_FRAGS / g)) * FRAG_MS:.0f} ms)")
    print(f"   baseline train = {TRAIN_FRAGS * FRAG_MS:.0f} ms")
    print("   NOTE: the 700 ms budget is base->tractor HYDRAULIC CONTROL")
    print("   latency, not image-frame latency. A 13-fragment image train is")
    print("   already 1521 ms and is not under that budget. What parity and")
    print("   CR4/8 actually spend is AIRTIME, which competes with the")
    print("   command slots that control latency depends on -- so the cost")
    print("   lands on control margin, not on image latency.")
    print("   CORRECTION: the campaign recorded CR4/8 as '~37% airtime'.")
    print(f"   Measured against the same ToA function: {100 * (t48 / t45 - 1):+.0f}%.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
