# W1-11 ping_pong 2026-05-22 16:51 — FAIL (BLOCKED by P8)

**Verdict:** `RX_PAIR_FAIL_5_GATES`
**Reason this evidence dir exists:** *not* an RF/hardware fault — a
firmware-vs-host-probe contract regression introduced by commit `d4dfcb8`
(2026-05-20). This was the first ping_pong run after today's HC-04
reflash uploaded the post-FHSS `firmware.bin` to both L072 boards.

## One-liner

Every cycle: `sx1276_tx_begin()` → `sx1276_fhss_next_channel()` returns
non-OK (scheduler not armed) → `HOST_ERR_PROTO_FORBIDDEN` on every
`TX_FRAME_REQ`. RX side: symmetric refusal → `HOST_FAULT_CODE_RX_SCAN_FAILED`
0x20 / 0x21 at listener start.

## Decoded smoking gun

- ERR_PROTO `10 01 08 00 00`
  → offending_type=0x10 (`HOST_TYPE_TX_FRAME_REQ`),
    err_code=0x08 (`HOST_ERR_PROTO_FORBIDDEN`).
- RX fault 0x0D = `HOST_FAULT_CODE_RX_SCAN_FAILED`
  (`murata_l072/include/host_types.h:198`).

## Why it is *not* hardware

Same boards, same probe script, **pre-FHSS firmware**, 2026-05-20 18:38 →
`radio_tx_ok=100/100`, PONG 97/100, RSSI -114 dBm. See
`../W1-11_pingpong_2026-05-20_183815/`.

## What to do

Do **not** re-run RF tests with the current probes against the current
`firmware.bin`. See:

- `../../AI NOTES/2026-05-22_W1-11_RF_Blocked_FHSS_Gate_v1_0.md` — full
  diagnosis + 3 fix options.
- `../../AI NOTES/2026-05-21_Open_Problems_To_Fix_v1_0.md` — P8 entry.
- `../../TODO.md` — top banner.

Fix needs user direction (FCC-adjacent code paths).
