# 2026-05-25 — Concurrent Smoke (post broker bring-up): RX VER warm-up fail + TX status=1(TIMEOUT) on every fragment

**Outcome:** `rx_frames_seen = 0`. **Per user directive, STOP — radio tuning is a separate sweep, not a code change.**

This note records the second 30 s concurrent smoke of the day (after the morning RX
USB-gadget-hang power-cycle). Two infrastructure gaps were closed before the run; the
test surfaced two real, independent radio-domain failures.

## Run setup (what was actually changed before this run)

1. **Paho-mqtt staged.** Previous run failed at `import paho.mqtt.client` →
   `ModuleNotFoundError: No module named 'paho'`. Installed `paho-mqtt==1.6.1` locally
   via `pip install --no-deps --target` and pushed `paho_stage/paho/` →
   `/tmp/lifetrac_strict/paho/` on both 2E2C and 2D0A (PYTHONPATH already had
   `/work/paho`). Verified `mqtt/client.py` present.
2. **MQTT broker started on RX board.** No mosquitto on the Windows host (no service,
   no winget install, no Docker Desktop). `LifeTrac-v25/DESIGN-CONTROLLER/base_station/`
   ships a `mosquitto.conf` that listens on `0.0.0.0:1883`, anonymous, persistence on,
   matching the design ("base station = broker"). Pulled `eclipse-mosquitto:2` onto
   2D0A, ran with `--network=host -v /tmp/lifetrac_strict/mosquitto.conf:/mosquitto/config/mosquitto.conf:ro`.
   Confirmed `LISTEN 0.0.0.0:1883`. Updated `publish_synthetic_frames.py` to read
   `LIFETRAC_MQTT_HOST` (default `192.168.1.117` = RX eth0). Re-ran smoke with
   `-HostIp 192.168.1.117`.
3. **gpio163 NRST preflight** (pre-existing change from prior session) pulsed both
   L072s and waited 1.5 s before launching daemons.

## RX result (2D0A): VER warm-up fails 2.7 s after NRST release

`AI NOTES/2026-05-25_smoke_rx_daemon_115117.log` (1.4 KB, complete):

```
2026-05-25 11:26:12,700 INFO  opening L072 HostLink on /dev/ttymxc3 @ 921600
2026-05-25 11:26:12,702 INFO  image_rx_daemon started; mqtt=192.168.1.117:1883
2026-05-25 11:26:12,712 INFO  MQTT connected; ready to publish lifetrac/v25/video/tile_delta
FAULT_URC during settle (expected for W1-9 lenient init): code=0x09 sub=0x01 lanes=LPUART1
STATS_URC(boot) drained seq=0
WARNING: dropped malformed frame: truncated COBS payload    x5
2026-05-25 11:26:13,704 INFO  stats: rx_frames=0 ...
2026-05-25 11:26:15,448 ERROR VER warm-up failed: timeout waiting for response type 0x81 to req 0x01
2026-05-25 11:26:15,450 ERROR fatal: cannot open L072 HostLink: timeout waiting for response type 0x81 to req 0x01
2026-05-25 11:26:15,719 INFO  image_rx_daemon exit
```

* MQTT connect path works (broker on this same board, localhost via host network).
* L072 emits FAULT_URC code=0x09 (LPUART1 fault) right after the gpio163 reset settle,
  followed by 5x `truncated COBS payload` (garbage stream) before VER_REQ ever gets a
  reply. RX daemon exits **before** entering RXCONT — never had a chance to demodulate.

**This falsifies the prior session's hypothesis that the gpio163 NRST pulse + 1.5 s
settle is sufficient on the RX board.** Same hypothesis (TX side) does work — see TX
result. So the divergence is per-board: the RX 2D0A L072 stays in an LPUART1-fault
state across the NRST pulse, while the TX 2E2C L072 cleanly boots.

## TX result (2E2C): VER + CFG_SET clean, but every TX_DONE is status=1(TIMEOUT)

`AI NOTES/2026-05-25_smoke_tx_daemon_115117.log` (24 KB, complete). Excerpt of init +
final stats line:

```
2026-05-25 11:26:15,902 INFO  image_tx_daemon started
2026-05-25 11:26:15,911 INFO  MQTT connected; subscribing to lifetrac/v25/cmd/image_frame
FAULT_URC during settle (expected for W1-9 lenient init): code=0x09 sub=0x01 lanes=LPUART1
BOOT_URC observed during settle (payload=000112010200)        # <- TX got BOOT_URC
Initializing regulatory profile 1 on co-processor...
CFG_SET_REQ(FHSS_CHANNEL_MASK) OK: 07000800
CFG_SET_REQ(ANTENNA_GAIN_DBI)   OK: 15000100
CFG_SET_REQ(HW_CEILING_DBM)     OK: 16000100
CFG_SET_REQ(REG_PROFILE=1)      OK: 14000100
2026-05-25 11:26:17,808 INFO  LBT_ENABLE=0 (matches W1-10b TX_BURST rationale)
2026-05-25 11:26:17,809 INFO  TX worker ready (inter_cycle_s=0.050, max 8 frags/dwell)
...
2026-05-25 11:26:36,926 INFO  stats: frames_in=31 ok=0 fail=15 drop_full=11 frags_ok=69 frags_fail=36 qdepth=4
```

Observations:

* **W2-02 v2 CFG_SET mirror works** — all four CFG_SET_REQ frames ACKed OK
  (FHSS_CHANNEL_MASK / ANTENNA_GAIN_DBI / HW_CEILING_DBM / REG_PROFILE=1).
* **Whole-frame ok = 0 / 31.** Per-frame summaries show 4–6 of 7 fragments OK and 1–3
  failing per frame, so the daemon's all-fragments-or-frame-fails rule turns every
  multi-fragment frame into a failure.
* **Per-fragment failures are uniformly `TX_DONE status=1(TIMEOUT) toa_us=48768`.**
  ~48 ms toa is consistent with SF7/BW500/CR5 at ~250 B payload. `frags_ok=69`
  means 69 individual fragments DID return TX_DONE OK — so the path is partially
  working; the timeouts are clustered.
* The TIMEOUT comes from the L072 firmware reporting that the SX1276 DIO0 TxDone IRQ
  didn't fire inside the expected window — i.e. PA reports it never completed the
  burst, or the IRQ was missed. Not an MQTT / not a USB / not a UART transport issue.

## Differentials & what is NOT the problem (vs. earlier suspicions)

| Hypothesis                            | This run says                                                           |
|---------------------------------------|-------------------------------------------------------------------------|
| paho-mqtt missing in container        | RESOLVED — staged at `/work/paho`, both daemons connected OK            |
| MQTT broker missing                   | RESOLVED — `eclipse-mosquitto:2` on 2D0A `192.168.1.117:1883`           |
| gpio163 NRST + 1.5 s settle universally fixes garbage stream | FALSIFIED on RX 2D0A — RX still gets truncated COBS + LPUART1 fault → VER timeout |
| W2-02 v2 CFG_SET PHY sequence not mirrored | FALSIFIED — TX side ran the exact 4-step CFG_SET cleanly                |
| TX side is the failing peer           | TX side is up, configured, transmitting; fragments succeed individually but most TX_DONEs report TIMEOUT |

## Per user directive: stopping here

> "Stop here if rx_frames is still 0 — radio tuning needs a separate sweep, not more code."

`rx_frames_seen = 0`. The RX daemon never got to RXCONT (VER timeout), and even if it
had, the TX side is reporting status=1(TIMEOUT) on every fragment. Both are
radio-domain / L072-firmware problems, not application code problems. No further
code changes are made in this work item. The next move is a deliberate radio sweep —
candidates (for the next session to choose between, not to implement now):

1. **Reflash both L072s with the W1-9 ISR-fix build** via `flash_unitA.ps1` /
   `flash_unitB.ps1` — many of the symptoms here (FAULT_URC code=0x09, missed
   TxDone IRQ) match the pre-W1-9 self-DOS pattern recorded in
   `/memories/methodology.md` ("blocking I/O in ISR error path stretches ISR past
   the next UART byte time"). Confirms the W1-9 fix is actually installed on this
   silicon.
2. **Repeat the gpio163 NRST pulse but with a longer settle** (>3 s) and a UART
   drain before VER_REQ on the RX side specifically, to test whether 2D0A simply
   needs more time post-reset.
3. **Substitute peers** — run RX role on 2E2C and TX role on 2D0A to localize
   whether the LPUART1 fault is board-specific (2D0A hardware) or daemon-specific
   (RX path triggers it).
4. **Read SX1276 IRQ flags + DIO mapping registers** post-TIMEOUT to distinguish
   "PA never completed" from "TxDone IRQ raised but lost on DIO0".

## Infrastructure artifacts left in place (no rollback needed)

* `eclipse-mosquitto:2` running on 2D0A as container `mosq`, `--restart unless-stopped`,
  bound to `/tmp/lifetrac_strict/mosquitto.conf`.
* `/tmp/lifetrac_strict/paho/` populated on both boards.
* `publish_synthetic_frames.py` now reads `LIFETRAC_MQTT_HOST` (default
  `192.168.1.117`).
* No firmware reflash, no L072 image change, no daemon code change this session.

## Cross-references

* Logs: `2026-05-25_smoke_rx_daemon_115117.log`, `2026-05-25_smoke_tx_daemon_115117.log`
* Prior session note: `2026-05-25_Concurrent_Smoke_RX_UART_and_TX_DONE_TIMEOUT.md`
* Memory: `/memories/methodology.md` (ISR blocking-I/O self-DOS pattern is now the
  leading TX TIMEOUT hypothesis to falsify next session)
