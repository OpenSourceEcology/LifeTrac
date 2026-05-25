# 2026-05-25 — Phase C bring-up complete on both X8s; OTA gap is the next blocker

## TL;DR
After the 2026-05-24 consolation-plan session (where 2D0A was offline), a
full power cycle restored 2D0A. Today the **strict-path image daemons run
concurrently on both Portenta-X8s** through the host Mosquitto broker
(`192.168.1.79:1883`), with both L072 HostLinks open, the FCC FHSS
regulatory profile loaded on both sides, and the SX1276 confirmed in
`LORA_RXCONTINUOUS` (opmode=0x85) on the RX side.

The only remaining gap to a green Phase C smoke test is an **over-the-air
delivery failure**: TX 2E2C reports `frags_ok=2 frags_fail=1(TIMEOUT)` for
a 3-fragment frame, but RX 2D0A counts `rx_frames=0` for the entire
30-second window. This is a pre-existing W1-10b/W2-02 radio-tuning class
of problem, not a Phase C bring-up failure.

## What changed today
1. **2D0A recovery via full power cycle.** USB-C reseat was insufficient
   (matches the LmP 934-91 first-boot adbd-wedge pattern from
   `/memories/repo/lifetrac-x8-adb-reverse-broken.md`). Hot-plug only;
   never run `adb kill-server` on a first-boot X8.
2. **Re-staged `/tmp/lifetrac_strict/` on 2D0A.** `/tmp` is tmpfs; the
   power cycle wiped the prior staging. Pushed canonical sources:
   - `LifeTrac-v25/DESIGN-CONTROLLER/base_station/image_rx_daemon.py`
   - `LifeTrac-v25/DESIGN-CONTROLLER/base_station/lora_proto.py`
   - `LifeTrac-v25/DESIGN-CONTROLLER/base_station/image_pipeline/{__init__,frame_format,reassemble}.py`
   - `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py`
   - `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py`
   - `paho_bundle.tgz` (extracted in-place)
3. **RX daemon dry-run on 2D0A PASS** (15 s timeout):
   L072 HostLink opened on `/dev/ttymxc3@921600`, MQTT connected to
   `192.168.1.79:1883`, VER warm-up OK, FCC profile loaded
   (`REG_PROFILE_FCC_15_247_FHSS_50CH_BW250`), RX worker draining
   `RX_FRAME_URC` (0x91).
4. **Concurrent smoke test (both daemons, 30 s):**
   - TX 2E2C: `frames_in=1 ok=0 fail=1 frags_ok=2 frags_fail=1`
   - RX 2D0A: `rx_frames=0 frames_published=0`
5. **Patched `image_rx_daemon._open_link`** to explicitly read+write
   `SX1276_REG_OP_MODE` and force `LORA_RXCONT` (0x85) if not already
   there. This mirrors the autowake folded into
   `method_h_stage2_tx_probe_v2.rx_listen` and the standalone
   `w2_02_radio_wake_rxcont.py`. On the bench today the read showed the
   radio was *already* in 0x85 after the FCC profile load, so the patch
   is defensive — but it removes a future foot-gun if any prior probe
   cleanup left the chip in `LORA_SLEEP` (0x80).

## What is still broken (next session)
The TX path delivers L072 `TX_DONE` for 2 of 3 fragments, but the RX
side hears zero. Top candidates, in priority order:

1. **PHY profile mismatch.** TX uses `PHY_IMAGE = SF7/BW500` (from
   `firmware/common/lora_proto/lora_proto.c`). RX has only loaded the
   FCC FHSS *regulatory* profile (`BW250`). Air symbol rate/BW must
   match for demodulation. Probable fix: call
   `configure_phy_profile(link, PHY_IMAGE)` (or the firmware equivalent)
   on the RX side in `_open_link` before the RX worker spins.
2. **FHSS hop alignment.** Both sides enable
   `FHSS_CHANNEL_MASK=0x07000800` but without a shared `hop_counter`
   schedule, TX and RX hop independently. Cross-check the W2-02 v2
   orchestrator path, which achieves 100/100 — it likely pins both
   sides to a single channel.
3. **TX-side `TX_DONE TIMEOUT` for fragment 0.** The TX log shows
   fragment idx=0 status=1(TIMEOUT). Worth pulling the L072 stats
   counters to see whether the radio actually emitted symbols or just
   gave up at the LBT/airtime gate.

## Memory updates
- `/memories/session/2026-05-24-phase-c-consolation.md` — appended
  2026-05-25 entry with the staging recipe and the OTA-gap diagnosis.
- `/memories/repo/lifetrac-strict-path-image-daemons.md` (existing
  unchanged) is the canonical recipe; no edits needed.
- `/memories/repo/lifetrac-x8-adb-reverse-broken.md` (existing
  unchanged) — full power cycle (not USB reseat) confirmed as the only
  reliable adbd-wedge recovery.

## Files touched
- `LifeTrac-v25/DESIGN-CONTROLLER/base_station/image_rx_daemon.py` —
  added SX1276 autowake-into-RXCONT in `_open_link` plus the two new
  imports (`read_reg`, `write_reg`, `SX1276_REG_OP_MODE`,
  `SX1276_OPMODE_LORA_RXCONT`).
- This AI NOTE.
- No other repo files modified.
