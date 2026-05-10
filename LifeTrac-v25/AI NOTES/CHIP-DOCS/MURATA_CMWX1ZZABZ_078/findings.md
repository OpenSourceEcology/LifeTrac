# Findings

- Official Portenta Max Carrier product metadata lists Murata CMWX1ZZABZ-078 as the onboard LoRa module.
- The Murata STM32 ROM bootloader path is proven reachable from current setup (19200 8E1), including successful flash operations.
- On the Max Carrier, the L072 boot/reset control path is now pinned in bench notes to BOOT0 on PA_11 and NRST on PF_4.
- A post-boot PA_11-high test did not restore user-mode ingress, so BOOT0 alone is not the missing gate.
- Custom firmware diagnostic evidence shows one-way behavior in user mode:
	- outbound periodic URCs are visible on host,
	- inbound host bytes are not observed at firmware IRQ ingress.
- Forcing PA11 high post-boot was tested and confirmed insufficient to restore user-mode ingress.
