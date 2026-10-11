# Bench board state capture, 2026-10-10

These are read-only captures made with
[`capture_board_state.sh`](../../firmware/x8_lora_bootloader_helper/bench_tools/capture_board_state.sh)
through [`pull_board_state.sh`](../../firmware/x8_lora_bootloader_helper/bench_tools/pull_board_state.sh).
For the summary and open items, see [BENCH_BOARDS.md](../../firmware/x8_lora_bootloader_helper/bench_tools/BENCH_BOARDS.md).
The base was captured on 2026-10-04 in [`../board_state_2026-10-04/`](../board_state_2026-10-04/).

## Power cycle, 2026-10-10 (radios kept off)

The operator power-cycled both boards around 16:58Z.

| time (UTC) | board | event |
|---|---|---|
| 16:58:44 | tractor `2E2C1209DABC240B` | Booted. |
| 16:58:51 | tractor | `lifetrac-camera.service` stopped; `/dev/ttymxc3` had no holder. |
| 16:58:55 | tractor | L072 read `0x85 RXCONT` (listening). |
| 16:58:59 | tractor | Parked: `PARK_OK`. |
| 16:59:21 | tractor | Re-read `0x80 SLEEP`. |
| 17:01 | base `2D0A1209DABC240B` | Booted. It did **not** enumerate on USB/adb, so it was handled over ethernet ssh. |
| 17:01:21 | base | L072 read `0x85 RXCONT`. |
| 17:01:29 | base | Parked: `PARK_OK`. |
| 17:01:53 | base | Re-read `0x80 SLEEP`. |
| 17:03:50 | tractor | Capture finished; radio still `0x80 SLEEP`. |

## `tractor/`

Captured 16:59–17:03Z. The tractor's clock read 2026-09-21; it has no time
source, so the tarball is named with that date.

The folder holds:
- the numbered reports;
- the docker inspect JSONs, with environment values redacted;
- the `/opt/lifetrac` and `/home/fio` manifests;
- `files/etc` and `files/usr`;
- `files/home_fio`: the board-only files `wdog_regs.py`,
  `lifetrac_compact_at_probe.sh`, `at_probe.sh`, and the 09-15 L072 flash
  logs.

Withheld by the script: `gshadow`, `machine-id`, the WiFi profile
`5star.nmconnection` (it holds the WiFi password), and the Dropbear host key.
Removed by hand: `/etc/docker/key.json`; the script's filter now covers it.

In the private archive outside git
(`C:\Users\dorkm\Documents\LifeTrac-bench-archive\board_state_2026-10-10\`):
- the full tarball;
- `opt_lifetrac_no_secrets.tgz`;
- the docker images `lifetrac-tractor-x8:pre-rs13` `9bfbbc8d06cb` (the only
  off-board copy) and `2727dfd36f9f`.
