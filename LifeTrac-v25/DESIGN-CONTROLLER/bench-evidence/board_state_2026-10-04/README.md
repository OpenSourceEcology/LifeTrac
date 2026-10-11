# Bench board state capture, 2026-10-04

What this folder holds:
- Read-only state captures of the bench X8s, made with
  [`capture_board_state.sh`](../../firmware/x8_lora_bootloader_helper/bench_tools/capture_board_state.sh)
  through [`pull_board_state.sh`](../../firmware/x8_lora_bootloader_helper/bench_tools/pull_board_state.sh).
- The summary and open items: [BENCH_BOARDS.md](../../firmware/x8_lora_bootloader_helper/bench_tools/BENCH_BOARDS.md).

**`base/`: base `2D0A1209DABC240B`, captured 2026-10-04 19:51Z.** It had
rebooted on its own about 4.5 min after the 17:46Z `systemctl poweroff`.
Before the capture, its L072 was read in RXCONT (listening only) and parked to
`0x80` SLEEP (`PARK_OK`, re-read SLEEP).

The files:
- The numbered reports.
- The docker inspect JSONs, with environment values redacted.
- The manifests of `/opt/lifetrac` and `/home/fio`. Secrets are listed by name
  and size only.
- `files/etc` and `files/usr`, minus the Dropbear host key and `machine-id`,
  which were removed by hand.
- `files/home_fio`: the board-only files.
- `retained_topics.txt`.

The full tarball, the deployed tree without secrets, and the docker image
exports are in the private archive outside git (BENCH_BOARDS.md, *Where
everything is saved*).

**`tractor/`: tractor `2E2C1209DABC240B`.** Not captured yet; the board was
powered off.
