#!/usr/bin/env python3
"""Falsification probe: does CFG_SET_REQ(CFG_KEY_TX_POWER_DBM=N) actually
move RegPaConfig (SX1276 0x09)?

Per user methodology rule: a "fix" applied on top of an unverified
precondition is silently a no-op. Before trusting `walk_power`'s CSV
output, prove the firmware's cfg_apply_tx_power_dbm slot fires and the
SX1276 PA stage register actually changes.

Reads RegPaConfig at three powers: default, 2 dBm, 17 dBm. PA_BOOST
formula (datasheet 5.4.3): Pout = 17 - (15 - OutputPower) dBm, with
RegPaConfig = 0x80 | OutputPower (top bit selects PA_BOOST). So:
  +2 dBm  → OutputPower=0  → RegPaConfig = 0x80
  +14 dBm → OutputPower=12 → RegPaConfig = 0x8C
  +17 dBm → OutputPower=15 → RegPaConfig = 0x8F

Run on the X8 with sudo (needs /dev/ttymxc3 write access).
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import method_h_stage2_tx_probe_v2 as m


def read_pa_config(link) -> int:
    val, _raw = m.read_reg(link, 0x09, timeout=1.0)
    return val


def set_tx_power(link, dbm: int) -> None:
    link.request(m.HOST_TYPE_CFG_SET_REQ, m.HOST_TYPE_CFG_OK_URC,
                 bytes([m.CFG_KEY_TX_POWER_DBM, 0x01, dbm & 0xFF]),
                 timeout=1.0)


def main():
    link = m.HostLink("/dev/ttymxc3", "921600")
    try:
        m.drain_boot(link, 1.0)
        try:
            link.request(m.HOST_TYPE_VER_REQ, m.HOST_TYPE_VER_URC, timeout=1.0)
        except Exception as exc:
            print(f"WARN: VER warmup: {exc}")
        m.drain_pending(link, quiet_s=0.25, max_s=1.0)

        baseline = read_pa_config(link)
        print(f"BASELINE RegPaConfig=0x{baseline:02X}")

        results = {}
        for dbm in (2, 8, 14, 17):
            set_tx_power(link, dbm)
            # Small settle so cfg_apply has run.
            import time
            time.sleep(0.05)
            val = read_pa_config(link)
            # Decode: bit7 = PA_BOOST; bits[3:0] = OutputPower; expected
            # PA_BOOST dBm = OutputPower + 2.
            pa_boost = bool(val & 0x80)
            output_power = val & 0x0F
            inferred_dbm = output_power + 2 if pa_boost else None
            results[dbm] = (val, pa_boost, output_power, inferred_dbm)
            print(f"set TX_POWER_DBM={dbm:>2} -> RegPaConfig=0x{val:02X} "
                  f"PA_BOOST={pa_boost} OutputPower={output_power} "
                  f"inferred_dbm={inferred_dbm}")

        # Verdict.
        unique_vals = {v[0] for v in results.values()}
        if len(unique_vals) < 3:
            print(f"__FALSIFICATION__=FAIL (only {len(unique_vals)} distinct "
                  f"PaConfig values across 4 power settings: {unique_vals})")
            print("  CFG_SET_REQ(TX_POWER_DBM) IS NOT MOVING PaConfig — "
                  "walk_power CSV would be misleading.")
            return 2
        # Per-setting check: inferred dBm should equal requested.
        mismatches = [(req, inf) for req, (_, _, _, inf) in results.items()
                      if inf != req]
        if mismatches:
            print(f"__FALSIFICATION__=PARTIAL (mismatches: {mismatches})")
            print("  PaConfig is moving but the dBm mapping disagrees with "
                  "the simple PA_BOOST = OutputPower + 2 formula. May be "
                  "fine if firmware uses RegPaDac high-power mode (0x4D).")
            return 1
        print("__FALSIFICATION__=PASS — CFG_SET_REQ(TX_POWER_DBM) moves "
              "RegPaConfig exactly as expected on PA_BOOST. walk_power CSV "
              "power column is trustworthy.")
        return 0
    finally:
        try:
            m.sleep_radio_safely(link, "falsify_tx_power_paconfig")
        except Exception as exc:
            print(f"WARN: sleep_on_exit: {exc}")
        link.close()


if __name__ == "__main__":
    sys.exit(main())
