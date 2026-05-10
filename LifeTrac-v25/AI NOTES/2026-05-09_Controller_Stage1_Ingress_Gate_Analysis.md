# Controller Stage 1 Ingress Gate Analysis

- ROM bootloader ingress is confirmed on the L072 path.
- User-mode ingress still fails after BOOT0, NRST, and post-boot `PA_11` tests.
- The failure now looks like a carrier-side routing or flow-control gate, not a simple reset-state problem.
- The next discriminating bench step is to scope or force RTS/CTS, then trace the Max Carrier/H747 route pins that own the UART path.
- Reference docs to keep using: ABX00043 schematics, full pinout, and user manual for topology only.