# HIL harness — Wave 4 bench tooling

**Scope.** Operational tooling that wraps the [`HIL_RUNBOOK.md`](../HIL_RUNBOOK.md)
prose procedures into a uniform, scriptable surface so day-1 with hardware is
"fill in three values and run" instead of "write the harness." Each W4-XX gate
gets a PowerShell harness skeleton with the relevant setup checklist, the
firmware-deploy command, the expected serial banner, the operator prompts, and
the JSONL result line — all in one place.

**This directory does NOT replace the runbook.** The runbook remains the
authoritative procedure; the harnesses are thin wrappers that enforce the
artifact-collection conventions of [`HIL_RUNBOOK.md`](../HIL_RUNBOOK.md) §0.3
and produce machine-readable result lines that the dispatcher aggregates.

## Layout

```
hil/
  README.md             # this file
  results_schema.json   # JSON Schema for one bench-evidence/*/results.jsonl line
  _common.ps1           # shared functions (firmware SHA, prompt, JSONL append)
  dispatch.ps1          # reads bench-evidence/, prints next-gate / progress
  w4-pre_board_bringup.ps1
  w4-00_lora_dual_portenta.ps1
  w4-01_estop_latency.ps1
  w4-02_link_tune_walkdown.ps1
  w4-03_m7_m4_watchdog.ps1
  w4-04_modbus_estop.ps1
  w4-05_proportional_ramp.ps1
  w4-06_mixed_mode_skip.ps1
  w4-07_boot_phy.ps1
  w4-08_camera_keyframe.ps1
  w4-09_async_tx_irq.ps1
  w4-10_fleet_key_provisioning.ps1
```

## Workflow

1. Plug in the hardware and complete the relevant setup from
   [`HIL_RUNBOOK.md`](../HIL_RUNBOOK.md): W4-pre and W4-00 use their
   minimal laptop/two-carrier setup; W4-01 and later use the full §0 common
   bench setup.
2. Edit the serial-port placeholders at the top of `_common.ps1` for your bench:
   * `$Script:HIL_HANDHELD_PORT` — handheld MKR WAN 1310 serial port (e.g. `COM7`)
   * `$Script:HIL_TRACTOR_PORT`  — Portenta H7 serial port (e.g. `COM8`)
   * `$Script:HIL_OPTA_PORT`     — Opta serial port (e.g. `COM9`)
   * `$Script:HIL_BASE_PORT`     — second Portenta Max Carrier serial port for W4-00 (e.g. `COM10`)
3. Pick the next gate via `pwsh ./dispatch.ps1` (prints next gate to run + progress).
4. Run the harness, e.g. `pwsh ./w4-pre_board_bringup.ps1 -Operator "alice" -SubGate usb`.
5. The harness walks you through the runbook procedure, captures evidence under
   `bench-evidence/W4-XX/`, and appends one JSON line per run to
   `bench-evidence/W4-XX/results.jsonl`.
6. Re-run `dispatch.ps1` to see updated counts and the next recommended gate.

## Result-line schema

Every harness writes one JSON object per bench run to
`bench-evidence/W4-XX/results.jsonl`. The schema is locked in
[`results_schema.json`](results_schema.json) and pinned by the SIL test
[`base_station/tests/test_hil_harness_completeness_sil.py`](../base_station/tests/test_hil_harness_completeness_sil.py).

## Sign-off

A gate flips to `[x]` in [`TODO.md`](../../TODO.md) when:

* Its `results.jsonl` contains the required number of `"result":"PASS"` rows
  (per the runbook's per-gate pass criterion).
* The dispatcher's final report shows the gate as `CLOSED`.
* Two operators have countersigned `bench-evidence/W4-XX/NOTES.md`.

The dispatcher's `-Report` mode emits a summary table that becomes the
"Wave 4 evidence package" appendix to [`SAFETY_CASE.md`](../SAFETY_CASE.md).
