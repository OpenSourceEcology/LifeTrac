=== W1-10b / W1-11 radio-link latency report ===
Evidence dir: C:\Users\dorkm\Documents\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\bench-evidence\W1-10b_rx_pair_2026-05-11_231144
TX_DONE rows parsed: 1000  (status==OK: 1000)
RX_FRAME rows parsed: 999  matched-to-TX-by-payload: 999

--- Time-on-air (LATENCY_BUDGET §1 row #2) ---
  toa (firmware-reported)          n=1000 min=  30.85 p50=  30.85 p95=  30.85 p99=  30.85 max=  30.85 ms
  predicted SF7/BW125 ~30.0 ms   measured-vs-predicted: +2.8%

--- Host-to-host TX-confirm (rows #1 + #2 + #3 + LPUART) ---
  elapsed_ms (TX_FRAME_REQ -> TX_DONE_URC) n=1000 min=  42.40 p50=  51.80 p95=  56.20 p99=  57.80 max=  60.80 ms
  budget for this segment: ToA + ~5.0 ms host overhead = ~35.0 ms

--- RX inter-arrival jitter (cross-check) ---
  rx_iat (L072 hw timestamp_us)    n=998  min=  93.06 p50= 102.83 p95= 107.86 p99= 108.42 max= 206.24 ms
  expected ~= TX inter_cycle_s + 1x ToA jitter

--- Estimated radio-link RTT (bench-tier surrogate) ---
  rtt_p50_estimate_ms = 2 * elapsed_p50 = 103.60 ms
  rtt_p99_estimate_ms = 2 * elapsed_p99 = 115.60 ms
  W4-00(b) target  = 2*toa_p50 + 30 ms slop = 91.70 ms (+/- 5% = 4.58 ms)
  W4-00(b) verdict  : OUTSIDE_BUDGET

--- Reconciliation vs LATENCY_BUDGET.md §1 ---
  This bench tier covers ONLY rows #1-#5 of the budget (encode +
  air + demod + AES + IPC). Rows #6-#13 (M4 100 Hz quant, Modbus,
  Opta SSR, coil ramp, spool shift, cylinder pressure) are NOT in
  scope here and add the remaining ~70-160 ms typical / ~580 ms
  worst case to the end-to-end stick->hydraulic figure.

