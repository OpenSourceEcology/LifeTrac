"""Bench health probe: hold /dev/ttymxc3 open and retry STATS.

The L072 wedges (TX-line break storm -> ISR livelock) when it boots with
the host UART CLOSED, so a one-shot probe after a reset usually misses.
This holds the port open and retries across the reset window — run it
CONCURRENTLY with a backgrounded SWD reset, exactly as
run_live_radio_monitor.ps1 sequences the TX daemon.

Prints the RS-11.5 discriminator counters when present: their existence
proves the instrumented firmware is the one in flash.
"""
import sys
import time

sys.path.insert(0, "/work")

from method_h_stage2_tx_probe_v2 import HostLink, fetch_stats, drain_boot  # noqa: E402

KEYS = ("radio_state", "radio_rx_ok", "radio_crc_err", "radio_dio0",
        "radio_tx_ok", "tx_fifo_rb_ok", "tx_fifo_rb_bad", "tx_done_early",
        "host_parse_ok", "host_parse_err", "host_rx_ring_ovf")

link = HostLink("/dev/ttymxc3", "921600")
try:
    drain_boot(link, settle_s=0.5)
except Exception as exc:                                  # pragma: no cover
    print(f"drain_boot: {exc}")

last = None
for attempt in range(1, 16):
    try:
        stats = fetch_stats(link)
    except Exception as exc:
        last = exc
        time.sleep(2.0)
        continue
    print(f"STATS-OK (attempt {attempt})")
    for key in KEYS:
        print(f"  {key}={stats.get(key, 'ABSENT')}")
    rs115 = "tx_fifo_rb_ok" in stats
    print(f"RS115-INSTRUMENTED-FIRMWARE={'YES' if rs115 else 'NO'}")
    sys.exit(0)

print(f"STATS-FAILED after retries: {last}")
sys.exit(1)
