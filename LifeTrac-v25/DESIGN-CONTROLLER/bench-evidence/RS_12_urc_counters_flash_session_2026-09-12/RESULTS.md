# RS-12 flash session — `rx_urc_lost` / `rx_pretx_drained` on air (2026-09-12)

**Status: complete. Predictions A and C failed; the loss site is upstream
of the URC path. The same evening the follow-up
(`../RS_12_11_command_timing_2026-09-12/`) fixed the camera-path loss on
the host (RS-12.11: 3.1 % → 1.0 %) and refuted the FIFO-coalescing
hypothesis proposed at the end of this document (`rx_fifo_skip` = 0 with
29 penultimate losses, RS-12.10). Read that directory for the current
state; this one is the morning's record.**

## Firmware on the boards

| board | build | md5 | flag | flashed |
|---|---|---|---|---|
| base 2D0A1209DABC240B | PR #116 @ 9dc4abbc, **bench build** `EXTRA_CFLAGS=-DHOST_ALLOW_REG_WRITE_DIAG=1` (23,784 B) | `67a4c0a492356792e73533b2976aa480` | diag register writes allowed | 2026-09-12 16:11–16:12 UTC, Verify OK |
| base (earlier the same day) | PR #116 @ 9dc4abbc, production build (23,788 B, committed `build/firmware.bin`) | `dbc79a6249434f66cd6f58f3bf8bdac7` | `HOST_ALLOW_REG_WRITE_DIAG=0` | 2026-09-12 ~15:49 UTC, Verify OK — **superseded** |
| tractor 2E2C1209DABC240B | PR #116 @ 9dc4abbc, **bench build** (same binary as the base) | `67a4c0a492356792e73533b2976aa480` | diag register writes allowed | 2026-09-12 after the legs (tractor clock unsynced: log says Aug 30 10:34), Verify OK, `REVIVE_MODE=reboot` — deliberate reboot, no oops; legs A–C flew on the 07-25 build (`1f69131235ca2d865e815304c7e7e578`) |

Tractor flash (after the legs, so the legs' tractor side is the 07-25
build): same pipeline with `REVIVE_MODE=reboot` — prep, flash, Verify
OK (`flash_rc=0`), revive up to `openocd reset run`, magic-close after 6
pets, `systemctl reboot`; board back in 89 s, no kernel oops
(`flash/tractor_pipeline_stamped.log`, `flash/tractor_wdt_pet.log`,
`flash/flash_pipeline_run3_tractor_bench_build.log`). After the reboot
the production `tractor-camera` container came back holding the UART
(health probe timeout, first park attempt `PARK_FAIL … timeout … req
0x01`); `docker stop tractor-camera` and the probe answered on attempt
1: `RS12-URC-COUNTERS=YES` (`flash/tractor_health_after_bench_build_flash.txt`),
with `host_parse_err`/`host_rx_ring_ovf` in the hundreds of thousands
from the container's traffic — the documented 08-08 symptom, not a
firmware fault. Both boards now run the same binary.

Why two base flashes: the committed production binary carries the F9
register-write gate (`host_reg_gate.c`, 2026-07-30) with
`HOST_ALLOW_REG_WRITE_DIAG=0`, so the bench carrier pin (`-ForceFrfHz`,
RegFrf 0x06–0x08) and `channel_survey_sniff.py` are refused with
`ERR_PROTO FORBIDDEN detail=6`. The legs need the 927.5 MHz carrier on
both radios (the tractor's 07-25 build has no gate and accepts it), so
the base was re-flashed with the bench build. Production flag stays 0;
`build/firmware.bin` in the PR is the production binary
(`firmware_bench_diag.bin` is untracked bench material).

Health snapshots (`flash/`): before the bench flash the base already
reported `RS12-URC-COUNTERS=YES` on the production build
(`base_health_before_bench_build_flash.txt`); after the bench flash
`RS12-URC-COUNTERS=YES`, all counters 0, `radio_state=4`
(`base_health_after_bench_build_flash.txt`). Tractor after its reboot:
`RS115-INSTRUMENTED-FIRMWARE=YES`, `RS12-URC-COUNTERS=NO`, as expected
for the 07-25 build (`tractor_health_after_reboot_old_build.txt`).

Same-day channel check on the flashed base (bench build, FRF write
accepted): **927.5 MHz 0 hot / max −94 dBm in 60 s** (949 samples),
`flash/spot_check_927p5_base.txt`. Back to clean after the single −72 dBm
hit of 2026-09-07.

## Three unplanned reboots, all explained

1. **Base, ~15:50 UTC, during `revive_bridge` of the production-build
   flash.** No persistent logs existed yet; the flash had verified.
2. **Tractor, ~15:57 UTC, ≈60 s after a `wdt_pet.sh start/stop` test.**
   Cause: the old `wdt_pet.sh` closed `/dev/watchdog0` without the magic
   `'V'` (its header claimed imx2 does not support magic close — wrong:
   the driver advertises `WDIOF_MAGICCLOSE` and the kernel core honours
   it). Without `'V'` the watchdog stays *active* with no petter; the
   base logged `watchdog: watchdog0: watchdog did not stop!` and
   survived once (kernel 6.1.24), the tractor (kernel 5.10.93) rebooted
   one timeout later. Fixed in `wdt_pet.sh`: `'V'` before close, 10 s
   pets, every pet fsync'd to `/home/fio/wdt_pet.log`.
3. **Base, 16:12:05 UTC (uptime 1304.7 s), during `revive_bridge`
   step "[4/5] rmmod by name + reload" of the bench-build flash — with
   the petter alive (PET 6 at 1302.6 s, oops at 1304.7 s).** The
   fsync'd `/dev/kmsg` tail (`flash/kmsg_flash.log`) has it:

   ```
   Unable to handle kernel paging request at virtual address ffffffc00111c538
   Internal error: Oops: 0000000096000007 [#1] PREEMPT SMP
   Modules linked in: x8h7_drv(O+) ...
   CPU: 2 PID: 2893 Comm: insmod Tainted: G  C O  6.1.24-lmp-standard #1
   pc : irq_find_matching_fwspec+0x64/0x120
   Call trace: irq_find_matching_fwspec → of_irq_get → spi_probe → really_probe → … → __spi_register_driver
   ```

   Re-inserting `x8h7_drv.ko` after the openocd session faults in the
   SPI driver's probe on the base's 6.1 kernel; the petter's next pet
   (due 1312.6 s) never happened, so the kernel went down at the oops
   and the 60 s WDOG (armed by u-boot, `WDOG_B` → PMIC) power-cycled the
   board. Reboot 1 was the same step of the same script. **Not the
   watchdog petter, not the flash: the flash had verified (`flash_rc=0`)
   both times.**

Watchdog facts established on the way (both boards): WDOG1
`WCR=0x773d` — enabled by u-boot, 60 s, `WDT=1` (timeout asserts
`WDOG_B` into the PMIC, so `WRSR` reads `POR=1` after *any* of these
reboots; the register cannot tell a watchdog reset from a power-up);
the kernel core keeps it alive from boot (`handle_boot_enabled=Y`,
`open_timeout=0`) until userspace opens `/dev/watchdog0`. No
`/sys/class/watchdog/*` attributes on either kernel; no `devmem`; host
python has no `mmap` — registers read from python inside the docker
image (`--privileged -v /dev/mem:/dev/mem`).

Pipeline change (`full_flash_pipeline.sh`, `revive_bridge.sh`):
`REVIVE_MODE=reboot` runs revive up to `openocd reset run` (the H7 back
in its firmware), hands the watchdog back, and reboots on purpose
instead of re-inserting the modules. Also: over SSH, root's PATH has no
`sbin`, so `rmmod` is not found and `prep_bridge` aborts before openocd
(run 1 today, harmless — `flash/flash_pipeline_run1_prep_abort_ssh_path.log`);
the wrapper now exports the path.

Instrumented run log: `flash/flash_pipeline_run2_bench_build.log`
(stdout), `flash/pipeline_stamped.log` (monotonic-stamped, fsync'd),
`flash/wdt_pet.log`, `flash/kmsg_flash.log`,
`flash/base_reboot2_forensics.txt` (post-reboot register dump and
module-load scripts).

## Legs

Pre-registered predictions (PR #116 body), base brackets with
`rs115_stats_probe.py`, 300 s each, 927.5 MHz, profile 2,
`-LogFragArrivals 1 -Archive`:

| leg | command | expect `rx_urc_lost` | expect `rx_pretx_drained` |
|---|---|---|---|
| A | `-TxFeed local -SynthBudgetB 3000 -KfRequestDisable 1 -NoParkLast 0` | ≈ timeouts | small |
| B | same, `-NoParkLast 1` | > 0 only if a second writer exists | small |
| C | `-TxFeed camera -KfRequestDisable 0 -NoParkLast 1` + `kf_inject.py 15 20` on first published frame | non-zero if the clobber path is real | > 0 |

Tractor brackets are recorded for completeness only (the harness
SWD-resets the tractor L072 at launch, and the tractor build has no
URC counters).

### Leg A — synth 13-frag, `-NoParkLast 0` (control) — **prediction FAILS**

Archive `radio_monitor_20260912_112220_6541c1ed` (`legs/legA_archive.txt`),
`params.txt`: `duration_s=300 kf_request_disable=1 no_park_last=0
tx_feed=local force_frf_hz=927500000`. Brackets `legs/legA_pre_base.txt`
→ `legs/legA_post_base.txt`; report `legs/legA_report.txt`.

| metric | value |
|---|---:|
| loss | **76/2384 = 3.2 %** |
| timeouts | **57** |
| frames published | 130 |
| train length | 13 modal (mixture 12 × 42, 13 × 149) |
| lost-index profile | idx 11 (penultimate of 13) = **27 of 78 attributed (35 %, uniform 8 %)** — the RS-12 signature, present as expected without the hold |
| radio Δ (base) | dio0 2558, rx_ok 2389, crc_err 41, tx_ok 115 |
| crc closure | Δcrc_err 41 = crc_dumps 41 |
| identity residue (dio0 − rx_ok − crc_err − tx_ok) | 13 |
| **`rx_urc_lost`** | **5** |
| **`rx_pretx_drained`** | **0** |
| host decode errors | `rx_decode_err=0` (URCs that arrive are intact) |

Predicted `rx_urc_lost ≈ timeouts` (57); measured 5. Edge coalescing in
`s_irq_events` accounts for well under a tenth of the loss, and the
TX-load FIFO clobber path never fired (`rx_pretx_drained` 0). The
penultimate lock is real and unchanged (35 %), so the mechanism that
produces it is not either of the two firmware paths the counters
instrument. Per the pre-registered fallback clause: the residual loss
is RF-level — the fragment never produces an RxDone on the base
(radio in TX or being re-armed when it arrives), and the fix belongs in
the base's command scheduler, not the URC path.

Where the 76 lost fragments went: the base radio demodulated 2389
frames (Δ`rx_ok`) and the host logged 2368 fragment-arrival URCs
(`grep -c frag_arrival`), so at most 21 demodulated frames did not
surface as fragment URCs — and that residue includes the tractor's
acks/replies to the base's 113 command sends (84 `TILE_STALE` + 17
`ENCODE_MODE` received at the tractor). The lost fragments therefore
never reached `rx_ok`: they were not demodulated. (The report's
"FIRMWARE DROP (Δrx_ok − host URCs) = 81" line uses the last periodic
`rx_frames` stats snapshot, 2308, not the full arrival count, and is
ack-contaminated as noted on 09-07; it is not a drop count.)

### Leg B — synth 13-frag, `-NoParkLast 1` (strict hold) — **no second writer**

Archive `radio_monitor_20260912_112923_6541c1ed` (`legs/legB_archive.txt`),
`params.txt`: `no_park_last=1`, otherwise as leg A. Brackets
`legs/legB_pre_base.txt` → `legs/legB_post_base.txt`; report
`legs/legB_report.txt`.

| metric | leg A (no hold) | **leg B (hold)** |
|---|---:|---:|
| loss | 76/2384 = 3.2 % | **34/2230 = 1.5 %** |
| timeouts | 57 | **33** |
| frames published | 130 | 143 |
| penultimate (idx 11 of 13) share | 27/78 = 35 % | **1/37 = 3 %** (uniform 8 %) |
| lost-index profile | 0:13 1:12 … 11:27 | flat: 0:1 1:5 2:2 3:4 4:7 5:3 6:4 7:3 8:2 9:2 10:2 11:1 12:1 |
| crc dumps / Δcrc_err | 41 / 41 | 26 / 26 |
| base command sends (tx_ok) | 115 | 120 |
| identity residue | 13 | 10 |
| **`rx_urc_lost`** | **5** | **1** |
| **`rx_pretx_drained`** | **0** | **0** |

Predicted "> 0 only if a second writer exists": measured 1 in 300 s.
There is no second writer at either instrumented site. The hold does
what it has always done on air — the penultimate lock disappears and
the residual 1.5 % is the flat, interference-shaped bench floor
(RS-11.6) — but it does so without moving either URC-path counter,
which means the lock it removes was never produced by URC coalescing or
by the TX-load FIFO clobber. Combined with leg A, the RS-12 loss site is
upstream of `rx_ok`: the penultimate fragment does not get demodulated
by the base when the final fragment rides 42 ms behind it. The 09-07
"URC contention from command/ack traffic" reading is withdrawn; the
counters it predicted stayed at 5 and 1.

### Leg C — camera, `-KfRequestDisable 0 -NoParkLast 1` + `kf_inject.py 15 20` — **clobber path never fires; losses sit right behind base transmissions**

Archive `radio_monitor_20260912_113635_6541c1ed` (`legs/legC_archive.txt`),
`params.txt`: `tx_feed=camera kf_request_disable=0 no_park_last=1`.
Injector started on the first published frame, 20/20 requests sent
(`legs/legC_kf_inject.txt`); the tractor logged 42 `REQ_KEYFRAME`, 88
`TILE_STALE`, 8 `ENCODE_MODE`. Railroad cab-view video verified
playing (2 h runtime, no autoplay chain possible in-leg); last frame on
the base website archived as `legs/legC_canvas_last_frame_20260912.jpg`.

| metric | 09-07 camera leg (same config) | **leg C** |
|---|---:|---:|
| loss | 40/980 = 4.1 % | **42/868 = 4.8 %** |
| timeouts | 14 | **25** |
| frames published | 583 | 565 |
| train lengths | 1 × 258, 2 × 359, 3 × 6 | 1 × 350, 2 × 255, 3 × 3 |
| publish sizes | none ≥ 600 B (max 497) | none ≥ 600 B (max 493) |
| lost-index profile | penultimate-of-2 12/14 | **idx 0 of 2 = 25/25** |
| base command sends | 145 | 171 (104 × 0x6c, 59 × 0x60, 8 × 0x63) |
| crc dumps / Δcrc_err | 9 | 7 / 7 |
| **`rx_urc_lost`** | n/a | **6** |
| **`rx_pretx_drained`** | n/a | **0** |

Predicted `rx_pretx_drained > 0` "if the FIFO-clobber path is real on
the base": 0, for the third leg running, under the heaviest command
plane of the campaign. The drain-before-TX branch never had anything to
drain. Retained as a guard (it is cheap and correct), but it is not a
fix for anything observed.

## Where the losses actually are — base TX coincidence (`tools/rs12_deaf_join.py`)

For every fragment the reassembler never saw, the expected arrival is
placed from a neighbour ± the leg's pacing, and the distance to the
nearest base `command TX … OK (on air)` line is measured; received
fragments give the baseline. Every lost fragment in all three legs (and
in the 09-07 leg) has a `TX_DONE status=0` in the tractor log — they
were transmitted and lost on the base side, upstream of `rx_ok`.
(`legs/deaf_join_legsABC.txt`)

| leg | lost, placeable | within ±150 ms of a base TX | received baseline | enrichment |
|---|---:|---:|---:|---:|
| A synth, no hold | 70 | 12 (17 %) — **none of the 10 penultimates** | 8.3 % | 2.1× |
| B synth, hold | 37 | 1 (3 %) | 5.8 % | 0.5× |
| **C camera + injector, hold** | **25** | **21 (84 %)** — 20 of 24 idx-0 losses; dt cluster −200…−30 ms, mode −150…−100 ms | 7.9 % | **10.7×** |
| 09-07 camera + injector, hold | 16 | 12 (75 %) — 12 of 13 "penultimates" | — | 8.7× |

Two mechanisms, now separated:

- **M2 — base command-plane deafness.** A fragment that arrives up to
  ~200 ms after a base command transmission is not demodulated. Under a
  heavy command plane (legs C and 09-07: 145–171 sends per 300 s) this
  is essentially all of the loss, and on 2-fragment camera trains it
  lands on the first fragment (the command fires in the train gap, the
  next train's first fragment arrives while the base is still in TX or
  re-arming) — which is what 09-07 read as "penultimate-of-two" and
  attributed to URC contention. That reading is withdrawn. In leg A the
  same mechanism accounts for the train-start losses (idx 0/1, 12 of
  70). The fix is in the base's command scheduler / TX→RX turnaround,
  exactly the pre-registered fallback; the hold does not touch it (leg
  C is under the hold).
- **M1 — the RS-12 penultimate-of-N lock, `-NoParkLast 0` only.** Leg
  A: 27 of 78 attributed losses at idx 11 of 13, none base-TX-coincident,
  all transmitted, none demodulated, `rx_urc_lost` 5, `rx_pretx_drained`
  0; leg B (hold): 1 of 37. The hold removes it. Its site is still not
  instrumented — see the correction below.

## Correction to PR #116: the edge counter is blind to the case it was built for

`sx1276_rx_service` clears `RegIrqFlags` only after it has read the
FIFO (radio/sx1276_rx.c:158, :304). DIO0 (RxDone) is a level that stays
high until that clear, so a second packet completing while the first
is still unserviced produces **no second rising edge**, and
`s_dio0_edges` counts one. Two RxDones before one main-loop pass is
precisely the case the PR body called "the only place that can see
them", and it is invisible to the edge counter unless the flags happen
to be cleared between the two completions. `rx_urc_lost` therefore
bounds only a narrow slice of coalescing (the 5 / 1 / 6 seen). The SX1276
FIFO semantics make the outcome the one observed: `FifoRxCurrentAddr`
points at the **last** packet, so an unserviced pair yields the final
fragment and drops the penultimate. With the final riding 42 ms behind
the penultimate (`-NoParkLast 0`), the two completions are ~80 ms apart
and a single long pass straddles them; with the hold they are ~120 ms
apart and the lock disappears (leg B). This is consistent with every
number in this session, but it is a hypothesis, not a measurement.

Instruments that would measure it (RS-12.10): (1) `rx_fifo_skip` — in
`rx_service`, compare `FifoRxCurrentAddr` with the previous packet's
`current + length (mod 256)`; a mismatch is a packet that completed
unserviced; (2) `loop_pass_max_us` / a pass-duration histogram on the
base, to see what holds a pass past ~80 ms after a fragment (the
synchronous 271 B URC send is ~24 ms at 115200; something else is in
that pass). Both are a few lines and read through the existing
additive stats tail.

## Verdicts

- **NO_PARK_LAST default: still do not flip.** The hold removes M1 but
  is irrelevant to M2, which is the loss that shows up under real
  command traffic; flipping the default would trade a 3.2 % → 1.5 %
  synth improvement for nothing on the camera path while the command
  scheduler is the actual defect.
- **`rx_urc_lost` / `rx_pretx_drained`: keep the wire fields and the
  drain guard; downgrade the claim.** They discriminated what they could
  (no second writer at the two instrumented sites) and their small
  values are now explained.
- **Next firmware step is RS-12.10 (two counters above), not a URC
  double buffer.** Next host step is the base command scheduler: do not
  fire a command when a fragment is due within (command ToA +
  turnaround), or measure and shorten the TX→RX re-arm.
