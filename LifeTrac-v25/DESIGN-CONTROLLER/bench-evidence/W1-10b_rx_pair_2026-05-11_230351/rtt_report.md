=== W1-10b / W1-11 radio-link latency report ===
Evidence dir: C:\Users\dorkm\Documents\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\bench-evidence\W1-10b_rx_pair_2026-05-11_230351
TX_DONE rows parsed: 200  (status==OK: 200)
RX_FRAME rows parsed: 199  matched-to-TX-by-payload: 199

--- Time-on-air (LATENCY_BUDGET §1 row #2) ---
  toa (firmware-reported)          n=200  min=  30.85 p50=  30.85 p95=  30.85 p99=  30.85 max=  30.85 ms
  predicted SF7/BW125 ~30.0 ms   measured-vs-predicted: +2.8%

--- Host-to-host TX-confirm (rows #1 + #2 + #3 + LPUART) ---
  elapsed_ms (TX_FRAME_REQ -> TX_DONE_URC) n=200  min=  43.10 p50=  55.40 p95=  56.50 p99=  57.60 max=  60.40 ms
  budget for this segment: ToA + ~5.0 ms host overhead = ~35.0 ms

--- RX inter-arrival jitter (cross-check) ---
  rx_iat (L072 hw timestamp_us)    n=198  min= 146.42 p50= 157.01 p95= 158.28 p99= 161.52 max= 306.60 ms
  expected ~= TX inter_cycle_s + 1x ToA jitter

--- Estimated radio-link RTT (bench-tier surrogate) ---
  rtt_p50_estimate_ms = 2 * elapsed_p50 = 110.80 ms
  rtt_p99_estimate_ms = 2 * elapsed_p99 = 115.20 ms
  W4-00(b) target  = 2*toa_p50 + 30 ms slop = 91.70 ms (+/- 5% = 4.58 ms)
  W4-00(b) verdict  : OUTSIDE_BUDGET

--- Reconciliation vs LATENCY_BUDGET.md §1 ---
  This bench tier covers ONLY rows #1-#5 of the budget (encode +
  air + demod + AES + IPC). Rows #6-#13 (M4 100 Hz quant, Modbus,
  Opta SSR, coil ramp, spool shift, cylinder pressure) are NOT in
  scope here and add the remaining ~70-160 ms typical / ~580 ms
  worst case to the end-to-end stick->hydraulic figure.

