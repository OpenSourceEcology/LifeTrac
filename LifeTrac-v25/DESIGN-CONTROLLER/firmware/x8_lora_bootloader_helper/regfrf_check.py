#!/usr/bin/env python3
"""Read RegFrf (SX1276 0x06/0x07/0x08) and print the actual centre frequency
the radio is currently programmed to transmit on. Lets a human verify the
firmware is on the regulator-correct ISM band before the next sweep.

Formula (datasheet 6.4): Fcarrier = Fxosc * Frf / 2^19,  Fxosc = 32 MHz.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import method_h_stage2_tx_probe_v2 as m

FXOSC_HZ = 32_000_000


def main():
    link = m.HostLink("/dev/ttymxc3", "921600")
    try:
        m.drain_boot(link, 1.0)
        try:
            link.request(m.HOST_TYPE_VER_REQ, m.HOST_TYPE_VER_URC, timeout=1.0)
        except Exception:
            pass
        m.drain_pending(link, quiet_s=0.25, max_s=1.0)
        msb, _ = m.read_reg(link, 0x06, timeout=1.0)
        mid, _ = m.read_reg(link, 0x07, timeout=1.0)
        lsb, _ = m.read_reg(link, 0x08, timeout=1.0)
        frf = (msb << 16) | (mid << 8) | lsb
        freq_hz = (frf * FXOSC_HZ) >> 19
        print(f"RegFrf = 0x{msb:02X}{mid:02X}{lsb:02X} (={frf})")
        print(f"Fcarrier = {freq_hz:,} Hz ({freq_hz / 1e6:.3f} MHz)")
        if 902_000_000 <= freq_hz <= 928_000_000:
            band = "US FCC Part 15.247 (902-928 MHz)"
        elif 863_000_000 <= freq_hz <= 870_000_000:
            band = "EU ETSI EN 300 220 (863-870 MHz) — NOT US-legal"
        elif 433_050_000 <= freq_hz <= 434_790_000:
            band = "EU 433 MHz SRD"
        else:
            band = f"UNKNOWN band — confirm regional legality before TX"
        print(f"Band: {band}")
        return 0
    finally:
        try:
            m.sleep_radio_safely(link, "regfrf_check")
        except Exception:
            pass
        link.close()


if __name__ == "__main__":
    sys.exit(main())
