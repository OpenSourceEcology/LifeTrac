# Open Questions

- What external route/control change between ROM and user boot states explains why ingress disappears only in user mode?
- Are there alternate UART destination paths on carrier hardware that can absorb host TX while still allowing L072 TX to host?
- Which additional GPIO/state experiments should be run next after PA11 to isolate the true gate net?
- Which carrier-side control net, beyond PA_11, changes between ROM bootloader and user firmware mode?
- Is the missing ingress gate tied to UART flow control, a mux/enable signal, or another board-level strap on the Max Carrier?
- What is the smallest discriminating next test: observe RTS/CTS, probe the H747 route pins, or capture the L072-side UART during the boot transition?
- The Max Carrier user manual helps confirm topology and switch behavior, but does it add anything beyond the schematic/pinout for the unresolved ingress gate?
- Next bench test to run: scope or force RTS/CTS while repeating the ROM-to-user transition and check whether host TX becomes visible only when flow control is held in the permissive state.
