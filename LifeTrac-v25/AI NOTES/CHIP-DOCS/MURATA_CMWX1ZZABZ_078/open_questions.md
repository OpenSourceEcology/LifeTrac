# Open Questions

- Which specific external net drives L072 host-RX on the Max Carrier path, and through which intermediary devices?
- Which control net beyond PA11 gates host-to-L072 ingress (if any)?
- Is hardware flow-control wiring (RTS/CTS side effects) involved in receive-path gating during user firmware mode?
- Next discriminating bench test: observe or override RTS/CTS on the carrier UART path during the user-mode probe and compare ingress success against the ROM-mode baseline.
