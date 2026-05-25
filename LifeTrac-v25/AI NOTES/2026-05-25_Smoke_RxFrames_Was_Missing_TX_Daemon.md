# 2026-05-25 — `rx_frames=0` Was Missing TX Daemon, NOT Firmware/RF

## TL;DR

The `rx_frames=0` baseline that drove WEEKS of investigation
(IQ-asymmetry hypothesis, FHSS-header silent-drop hypothesis,
RegInvertIQ flips, sniff-race hunting, all the way to planning a
diagnostic `HOST_TYPE_RX_DROP_URC`) was **never a firmware, RF,
modem, header, or UART bug**.

It was caused by `image_tx_daemon.py` being **absent on the TX
board's `/tmp/lifetrac_strict/` directory**. The TX container started
and immediately died with
`python3: can't open file '/work/image_tx_daemon.py': [Errno 2] No
such file or directory`, but the smoke script only checked the RX
log for `rx_frames=...` — so a TX that never emitted a single frame
looked indistinguishable from a fully-broken RX path.

## The decisive falsification

Built a minimal NRST-only orchestrator
(`run_rx_pair_nrst.ps1`) that runs `method_h_stage2_tx_probe_v2.py
--probe rx_listen` on 2D0A and `--probe tx_burst` on 2E2C — the
same hardware, same firmware, same UART, same SX1276 registers.

Result:

```
================ HEADLINE ================
  TX tx_ok               = 30/30
  RX rx_frames (URCs)    = 30
  RX radio_rx_ok_delta   = 30
  RX radio_crc_err_delta = 0
==========================================
VERDICT: RX path fully working.
```

30/30, RSSI ~ -92 dBm, SNR ~ 9 dB. The FHSS-header gate strips
correctly. The CRC is clean. The URCs reach the host.

Then re-ran the original `run_concurrent_smoke.ps1` against the
*same* firmware and saw:

```
=== TX DAEMON LOGS ===
python3: can't open file '/work/image_tx_daemon.py': [Errno 2] No
such file or directory

=== RX DAEMON LOGS ===
2026-05-25 17:07:33,285 ERROR image_rx_daemon: MQTT connect to
192.168.1.79:1883 failed: timed out
```

No TX frames were ever emitted. The "RX failure" was actually a TX
that never ran plus an RX that couldn't reach MQTT.

## What had hidden this

1. `run_concurrent_smoke.ps1` assumed the two daemons were
   pre-deployed under `/tmp/lifetrac_strict/`. They never were.
   `image_rx_daemon.py` survived on 2D0A from an earlier ad-hoc
   push; `image_tx_daemon.py` was simply missing on 2E2C.
2. The script's only success/failure decision was a regex for
   `rx_frames(?:_seen)?=([1-9]\d*)` in the RX log. A
   `MQTT connect failed` error in the RX log AND a missing-file
   error in the TX log both produced the same final report:
   `rx_frames=0` → "FAILURE: No frames demodulated".
3. The two boards have different `/tmp` content because nobody had
   a hermetic deployment step. The smoke was silently dependent on
   filesystem state from prior unrelated runs.

## Misdiagnoses caused

All of the following hypotheses were premised on
`rx_frames=0 ⇒ frames are being dropped after demod`:

- **IQ-asymmetry** (RegInvertIQ 0x33). Falsified on its own merits
  (writing `0x26` made sniff strictly worse) — but the falsification
  test was set up *because* the false symptom looked airtight.
- **FHSS-header silent-drop**. `sx1276_rx_service()` DOES silently
  drop on `lora_pkt_hdr_unpack()` failure, but with both peers
  sharing the same `SCHEMA_VER=1U` constant this could only be the
  culprit if the headers were structurally corrupt. They weren't.
- **`air_coupling_rssi_sniff.py` race**. Real bug (host clears IRQ
  flags before firmware reads them) but only affects observations
  taken *during* a sniff run — not the smoke baseline.
- **DIO0 / opmode / RXCONT autowake**. The probe already proves
  RXCONT autowake works.
- **Planned `HOST_TYPE_RX_DROP_URC` firmware patch**. Would have
  rebuilt + reflashed both peers to instrument a path that already
  works perfectly.

## Fix shipped

`LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_concurrent_smoke.ps1`
now pushes `image_rx_daemon.py`, `image_tx_daemon.py`,
`lora_proto.py`, `image_pipeline/`, and `method_h_stage2_tx_probe_v2.py`
to both boards before launching the daemons. Smoke is now hermetic
w.r.t. deployment state.

The MQTT broker reachability at `192.168.1.79:1883` is a separate
config issue (depends on whether a local broker is running) and is
not patched here.

## Lessons

1. **A "status field == 0" in a multi-stage test tells you nothing
   about which stage failed.** Always look at every stage's raw
   log before pivoting diagnosis. This is the *third* time this
   exact pattern has caught us — see `misdiagnosis.md` Method-G
   SYNC_OK=0 (INLRCR typo) and this one.
2. **Before suspecting firmware/RF, run a hermetic
   reference test on identical hardware with a known-good script.**
   The `--probe rx_listen` + `--probe tx_burst` pair on
   `method_h_stage2_tx_probe_v2.py` is exactly that reference. We
   had it the whole time and never ran it.
3. **Bench harnesses must be deployment-hermetic.** Any
   "did-you-deploy-it?" failure mode that looks like a hardware
   symptom will burn weeks. Push every dependency every run.
4. **Smoke verdict regexes must be paired with stage liveness
   checks.** "No TX frames were ever attempted" should produce a
   different headline than "TX frames sent, no RX URCs seen".
