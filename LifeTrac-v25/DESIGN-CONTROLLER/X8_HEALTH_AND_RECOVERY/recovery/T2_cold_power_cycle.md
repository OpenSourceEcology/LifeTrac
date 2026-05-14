# T2 — Cold power cycle

**Tier:** 2 (physical access required; no DIP changes)

**When to try:**
- Soft hang where adb is gone for more than ~60 s and HC-01 shows USB
  enumerated but adbd absent.
- HC-03 shows `srst 0` divergence (IOMUXC pad drift) — soft reboot will not
  clear this.
- After any session of camera USB-host wedges.

## Procedure

1. Unplug **both** the X8 PWR USB-C **and** the 12V barrel from the Max Carrier.
2. Wait ≥ 10 seconds (drain rails; required to reset i.MX IOMUXC).
3. Reconnect 12V first, then USB-C.
4. Wait 30–60 s for `adb devices` to show the board as `device`.
5. Run HC-01 → HC-02 → HC-04.

## Pass criteria

- HC-01 PASS (stable, no transient drop).
- HC-02 PASS (all 9 x8h7 modules, services active, gpio8/10/15 in expected state).
- HC-04 ≥ 5 cycles PASS_SYNC.

## Common failure modes (T2 insufficient)

| Symptom after T2 | Diagnosis | Escalate to |
|---|---|---|
| Board comes up, adbd starts, then dies seconds later (repeatable) | Wedge is **in eMMC boot itself** — every boot reproduces it. | T3a SDP reflash |
| Board never enumerates after reconnect | Hardware fault or cable | Try different cable / port; if still nothing, hardware-RMA candidate |
| HC-03 still shows `srst 0` after T2 | Rare; suggests a non-IOMUXC stuck pad (analog level on the gpio10 net) | Inspect carrier wiring; T3a |

## Verdict matrix

| Date | Board | Trigger | Outcome | Notes |
|---|---|---|---|---|
| 2026-05-12 | Board 1 (`2D0A`) | Camera-induced ci_hdrc-imx wedge | PARTIAL | Cleared previous USB hang, but later session reproduced same wedge after another camera attempt. |
| 2026-05-13 | Board 1 (`2D0A`) | Overnight disconnect (~12 h unpowered) | FAILED | adbd briefly visible at session start then died; same soft-hung Linux state as before. Escalate to T3a. |
| 2026-05-13 | Board 1 (`2D0A`) | Manual USB-C + 12V unplug (post-soft-reset-index) | PARTIAL | HC-01 + HC-02 PASS (uptime 1 min, all 9 x8h7 modules, services active, fw `0.0.5-next-53df799`). **HC-04 Stage 1: 3/3 FAIL_SYNC.** HC-03 post-preflight: `srst 1` (gpio10 cleared) but `cannot read IDR` (gpio8/15 SWD pads still wedged). Linux soft hang cleared; SWD-side IOMUXC drift survived T2. Escalate to T3a or longer-hold T2. |
| 2026-05-13 | Board 2 (`2E2C`) | (no T2 needed) | CONTROL | HC-04 Stage 1 3/3 PASS in 76 s — confirms pipeline + tooling healthy; failure on Board 1 is board-specific. |
| 2026-05-13 | Board 1 (`2D0A`) | Long-hold T2 (≥60 s, both rails) | **FAIL** | HC-01 + HC-02 PASS (uptime 1 min, 9 x8h7 modules, services active, /tmp wiped as expected). HC-03 post-preflight: gpio8/10/15 muxed correctly (`srst 1`) but DP IDR still unreachable — `bitbang_swd_read_reg(): JUNK DP read reg 0 = ffffffff` / `Error connecting DP: cannot read IDR`. HC-04 Stage 1 1/1 FAIL_SYNC (RUN_ID `T6_stage1_standard_quant_2026-05-13_154202_460-46560`). **T2 has now been proven insufficient at both ~10 s and ≥60 s holds for this wedge class.** Escalate to T3a. |
