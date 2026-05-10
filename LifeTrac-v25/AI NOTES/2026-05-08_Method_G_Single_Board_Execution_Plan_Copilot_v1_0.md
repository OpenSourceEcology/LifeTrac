# Method G Single-Board Execution Plan

**Date:** 2026-05-08
**Author:** GitHub Copilot (GPT-5.3-Codex)
**Version:** v1.0
**Scope:** Proceeding with Method G custom LoRa firmware work when only one Portenta+MaxCarrier stack is available.

## 1. Decision

Proceed now with a single-board loop.

You can continue closing Method G implementation risk without waiting for the second board by emphasizing:
- flash/recovery reliability,
- host-wire protocol robustness,
- runtime observability,
- repeatable evidence capture.

## 2. What changed in tooling

`full_flash_pipeline.sh` is updated for one-board default behavior:

1. `revive_bridge` is now optional and **off by default** (`RUN_REVIVE=0`) since it is diagnostic-only.
2. Post-flash runtime capture is **on by default** via `boot_and_listen_hold.sh` (`RUN_POST_LISTEN=1`).
3. Post-listen duration is configurable (`POST_LISTEN_SEC`, default `8`).
4. Exit code now reflects real stage failures in order: prep -> flash -> listen -> revive -> wdt stop.

Path:
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/full_flash_pipeline.sh`

## 3. Session recipe (single-board)

On X8:

```bash
echo fio | sudo -S -p '' \
  RUN_POST_LISTEN=1 POST_LISTEN_SEC=10 RUN_REVIVE=0 \
  bash /tmp/lifetrac_p0c/full_flash_pipeline.sh /tmp/lifetrac_p0c/hello.bin
```

Expected behavior:
1. watchdog pet starts,
2. bridge prep runs,
3. L072 flash+verify runs,
4. boot/listen captures UART output for 10s,
5. watchdog pet stops,
6. system may auto-reboot later; adb reattach is acceptable.

## 4. One-board weekly goals

1. Flash reliability envelope:
- 20 consecutive flash cycles with zero verify failures.
- Record cycle time distribution and failure modes.

2. Post-flash liveness contract:
- Require banner/tick signature in `rx.bin` on each cycle.
- Flag missing output as hard failure.

3. Host-wire stress without RF peer:
- Continue PTY/TCP loopback malformed-frame and chunked-I/O stress.
- Grow vector corpus for counter ownership and parser hardening.

4. Pre-RF readiness package:
- Build bench evidence folder with logs proving reproducibility.
- This shortens time-to-W4-00 once the second board arrives.

## 5. Exit criteria for adding second-board RF work

Move to two-board RF gates immediately when hardware arrives if all are true:

1. flash/verify success >= 95% over last 20 runs,
2. post-flash banner capture success >= 95%,
3. no unresolved watchdog/pipeline deadlocks in last 10 runs,
4. host-wire parser fuzz/loopback suite clean.

## 6. Notes

- This plan does not claim closure of W4-00 RF exchange gates; it minimizes schedule risk until the second board is available.
- Keep all pipeline artifacts (`pipeline.log`, `flash_run.log`, `openocd_boot.log`, `rx.bin`) per run for trend analysis.
