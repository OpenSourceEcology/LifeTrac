# L072 bench firmware kept with the RS-13.1 record

Everything here is a **bench build**: `mingw32-make bench`, with
`HOST_ALLOW_REG_WRITE_DIAG=1` so the host may write radio registers such as the
DTS carrier pin. None of it is the production binary. Production is the
tracked `firmware/murata_l072/build/firmware.bin` (md5 `589c1203…` on `main`),
which refuses those register writes. Flash only with
[FLASH_RUNBOOK.md](../../../firmware/x8_lora_bootloader_helper/FLASH_RUNBOOK.md)
(`REVIVE_MODE=reboot`, so expect an X8 reboot and a `/tmp` re-push), and only on
an explicit GO.

## What the boards run (`0c1bb0a9`)

- **Source and build.** `0c1bb0a9` is the RS-12.15 v2 bench build of the
  PR #125 head (`efd69f7d`; `aa74cde2` later changed only comments). PR #125
  merged as `f2474596`.
- **Byte-for-byte reproducible.** A clean `mingw32-make bench` of current
  `main` (`bb4a2071`) on 2026-10-04 gave md5 `0c1bb0a9573f…` again. The
  `firmware/murata_l072` source on `main` is unchanged since.
- **Radio profiles** (`include/host_cfg_keys.h`):
  - 0 = bench fixed 915 MHz;
  - 1 = FCC 15.247 **FHSS**, 50 channels at 250 kHz;
  - 2 = FCC 15.247 **DTS** at 500 kHz.

  The host switches between them at runtime with `CFG_SET(REG_PROFILE)`
  (`host/host_cfg.c`, `host/host_cfg_profile.c`).
- **FHSS features:**
  - 50-channel hop table and seeded permutation;
  - 200 ms slot clock;
  - scan/acquisition state machine and slot follower;
  - 400 ms per-channel legal dwell;
  - LBT;
  - the RS-12.15 v2 clock authority.
- **DTS features:** a single carrier at 500 kHz with a 950 ms airtime budget.
  The firmware never sets the DTS carrier itself; the host pins it with
  register writes, which only bench builds accept (RS-11.7).
- **The auto switch is not firmware.** `AutoRadioPolicy` in
  `base_station/web_ui.py` and the daemons' switch handshake run on the X8s,
  and send the profile change to the L072. All it needs from the firmware is
  the runtime profile switch, plus the carrier write for DTS. `0c1bb0a9` has
  both.
- **On air with this build:** leg V (RS-1.4, 2026-09-15) switched DTS → FHSS →
  DTS twice without a reflash. RS-13.1 flew both profiles. An in-band auto
  switch has not completed end to end on this build: leg V's FHSS switch
  reverted after the 45 s proof-of-life window (RS-12.19). The last completed
  switches were on 2026-07-26, on older firmware.
- **Not in any build (open firmware work, not lost):** the RS-12.15
  reverse-delivery fix for base → tractor commands on FHSS.

## Binaries in this folder

| file | md5 | size | what it is | where it came from |
|---|---|---|---|---|
| `firmware_bench_diag_0c1bb0a9.bin` | `0c1bb0a9573f813137f941dfa47177d0` | 24 860 B | **On both boards** since 2026-09-15 (leg U, leg V, all of RS-13.1). | PC build dir; rebuilt byte-identical from `main` |
| `firmware_bench_diag_5a160e4a.bin` | `5a160e4a8c9296c7d2e49727bdfb8880` | 24 860 B | RS-12.15 v2 as **flown in legs R and T** (both boards, 2026-09-14 → 09-15). Source `6feca2c0`. It differs from `0c1bb0a9` only in the trust tier given to a leading grid, a case never seen on air. | Recovered from the PC's local git object store (blob `d877970b`, reachable only from a reflog entry of the amended commit `4de3afa2`, which expires ~2026-10-14) |
| `firmware_bench_diag_e8ad8424.bin` | `e8ad842489d5acfc09f204c7807e4661` | 24 196 B | RS-12.10 corrected build: the **pre-clock-authority "old fw"** of the RS-12.15 A/B (legs L, Q, Q2, S), on both boards 09-12 → 09-14. Source `e02ab86e` (`main` `3a0cb524`). | The PC's `LifeTrac-oldfw` worktree build dir |
| `firmware_bench_diag_8c112e6f.bin` | `8c112e6f8b42109aab38a69f2b0aa147` | 24 180 B | First RS-12.10 build (PR #117, `df997023`), legs D/E on 2026-09-12. It has the STATS-tail serializer bug fixed in `e8ad8424`; do not fly it. | Recovered from an unreachable blob (`cb877771`) in the PC's git store |
| `firmware_bench_diag_43f0a74c_fhss_gap1500.bin` | `43f0a74cde4137dccf8e8943bf9c3acc` | 24 864 B | Branch `imp/fhss-authority` @ `2916601e` (on origin): streak gap 1000 → 1500 ms for A11. **Never flashed, rejected.** A gap of 1500 ms lets base commands, paced at ≥ 1.0 s, chain into authority. | Its worktree build dir |

## Every L072 build flashed to the bench, and where it is now

Based on the bench-evidence records, git history and a scan of the PC on
2026-10-04:

| build (md5) | when / legs | status |
|---|---|---|
| `0c1bb0a9` bench | 09-15 → now (U, V, RS-13.1) | **here** + reproducible from `main` |
| `5a160e4a` bench | 09-14 → 09-15 (R, T) | **here** (recovered) + source `6feca2c0` |
| `2ee69f9c` bench (RS-12.15 v1) | 09-14 (O, P) | binary lost; source `23ba5122` on `main` |
| `e8ad8424` bench | 09-12 → 09-14 (F, L, Q, Q2, S) | **here** + source `e02ab86e` |
| `8c112e6f` bench | 09-12 (D, E) | **here** (recovered) + source `df997023` |
| `67a4c0a4` bench | 09-12 | binary lost; source `9dc4abbc` on `main` |
| `dbc79a62` production | 09-12, base, ~20 min | tracked in git at `9dc4abbc` |
| RS-11.5 RF-switch build | 08-02 → 09-12 (RS-11.6/11.8, RS-12 August, RS-3.3) | probably tracked as `5bdfa0dc` at `85bd5f5e` (size and commit match; no md5 at flash time) |
| RS-11.5 leg D / leg G test builds | 08-02 | binaries lost; sources `87821c90` / `107634bb` on `main` |
| Batch 1 builds | 07-30 → 08-01 | tracked (`87d65070`, `faaee737`; size and commit match) |
| Run D build | 07-25 → 26 | tracked (`9493efb4` at `69df9cee`) |
| `1f691312` patch set | 07-25, a few hours | **missing**: no binary, and its exact source was never committed |
| 07-24 FHSS bring-up, May T6 / W1–W2 builds | 05 – 07-24 | T6 builds archived in their run dirs; the rest are sources on `main`. Builds before `d4dfcb86` (05-20) have no FHSS/DTS profiles |

No missing build has an FHSS, DTS or auto-switch capability that `0c1bb0a9`
lacks. The only firmware source not on `main` is the rejected
`imp/fhss-authority`.
