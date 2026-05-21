=== W1-10b / W1-11 radio-link latency report ===
Evidence dir: C:\Users\dorkm\Documents\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\bench-evidence\W1-11_pingpong_2026-05-20_183815
TX_DONE rows parsed: 100  (status==OK: 100)
RX_FRAME rows parsed: 98  matched-to-TX-by-payload: 98

--- Time-on-air (LATENCY_BUDGET §1 row #2) ---
  toa (firmware-reported)          n=100  min=  33.41 p50=  33.41 p90=  33.41 p95=  33.41 p99=  33.41 p999=  33.41 max=  33.41 ms
  predicted SF7/BW125 ~30.0 ms   measured-vs-predicted: +11.4%

--- Host-to-host TX-confirm (rows #1 + #2 + #3 + LPUART) ---
  elapsed_ms (TX_FRAME_REQ -> TX_DONE_URC) n=100  min=  43.40 p50=  55.90 p90=  57.30 p95=  57.90 p99=  60.80 p999=  61.00 max=  61.00 ms
  budget for this segment: ToA + ~21.0 ms host overhead = ~51.0 ms

--- RX inter-arrival jitter (cross-check) ---
  rx_iat (L072 hw timestamp_us)    n=97   min= 300.05 p50= 309.41 p90= 318.67 p95= 320.00 p99=3694.18 p999=3710.99 max=3710.99 ms
  expected ~= TX inter_cycle_s + 1x ToA jitter

--- Estimated radio-link RTT (bench-tier surrogate) ---
  rtt_p50_estimate_ms = 2 * elapsed_p50 = 111.80 ms
  rtt_p99_estimate_ms = 2 * elapsed_p99 = 121.60 ms
  W4-00(b) target  = 2*toa_p50 + 30 ms slop = 96.82 ms (+/- 5% = 4.84 ms)
  W4-00(b) verdict  : OUTSIDE_BUDGET

--- True ping-pong RTT (W1-11 L-1, host-driven echo) ---
  pingpong_rtt_ms                  n=97   min=  94.20 p50= 106.70 p90= 113.00 p95= 116.90 p99= 118.00 p999= 118.00 max= 118.00 ms
  W4-00(b) (true RTT) target = 96.82 ms (+/- 4.84 ms)
  W4-00(b) (true RTT) verdict: OUTSIDE_BUDGET

--- Reconciliation vs LATENCY_BUDGET.md §1 ---
  This bench tier covers ONLY rows #1-#5 of the budget (encode +
  air + demod + AES + IPC). Rows #6-#13 (M4 100 Hz quant, Modbus,
  Opta SSR, coil ramp, spool shift, cylinder pressure) are NOT in
  scope here and add the remaining ~70-160 ms typical / ~580 ms
  worst case to the end-to-end stick->hydraulic figure.

