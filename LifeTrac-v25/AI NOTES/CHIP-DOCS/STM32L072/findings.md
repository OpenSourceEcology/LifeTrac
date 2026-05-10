# Findings

- STM32L072 ROM-vs-user asymmetry is confirmed on board 2E2C1209DABC240B:
	- ROM bootloader session works on 19200 8E1.
	- User firmware does not ingest host bytes even when outbound diagnostics remain active.
- Firmware-level IRQ echo diagnostic was implemented and tested; no host pattern echo was observed in user mode.
- Parity mismatch is ruled out as primary root cause for Stage-1 ingress failure.
- PA11 forced-high post-boot does not resolve ingress, so PA11 alone is not the gating mechanism.
