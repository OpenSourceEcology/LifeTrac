"""RS-11.5 L072 counter readout. Run on the base X8 AFTER a bench run,
with the rx daemon stopped and the L072 NOT reset (docker stop leaves it
in RXCONT; counters live in RAM since the harness's launch-time NRST).

Splits the penultimate-fragment loss three ways:
  radio_rx_ok  ~= frags_tx            -> demodulated, dropped host-side
                                         (host_rx_ring_ovf should account)
  radio_rx_ok  ~= daemon rx_frames    -> never demodulated (PHY / radio
                                         state at train end)
  radio_crc_err ~= the shortfall      -> demodulated corrupt
"""
import sys

sys.path.insert(0, "/work")

from method_h_stage2_tx_probe_v2 import HostLink, fetch_stats, drain_boot  # noqa: E402

link = HostLink("/dev/ttymxc3", "921600")
drain_boot(link)
stats = fetch_stats(link)
for key in ("radio_rx_ok", "radio_crc_err", "radio_dio0", "radio_tx_ok",
            "host_rx_ring_ovf", "host_dropped", "host_queue_full",
            "host_parse_ok", "host_parse_err", "radio_state",
            "tx_fifo_rb_ok", "tx_fifo_rb_bad", "tx_done_early"):
    print(f"{key}={stats.get(key, 'ABSENT')}")
print("ALL:", {k: v for k, v in stats.items() if v})
