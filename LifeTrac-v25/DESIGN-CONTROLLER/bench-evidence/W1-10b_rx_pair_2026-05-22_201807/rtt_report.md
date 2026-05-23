=== W1-10b / W1-11 radio-link latency report ===
Evidence dir: C:\Users\dorkm\Documents\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\bench-evidence\W1-10b_rx_pair_2026-05-22_201807
TX_DONE rows parsed: 0  (status==OK: 0)
RX_FRAME rows parsed: 0  matched-to-TX-by-payload: 0

--- Time-on-air (LATENCY_BUDGET §1 row #2) ---
  toa (firmware-reported)          (no samples)
  predicted SF7/BW125 ~30.0 ms

--- Host-to-host TX-confirm (rows #1 + #2 + #3 + LPUART) ---
  elapsed_ms (TX_FRAME_REQ -> TX_DONE_URC) (no samples)
  budget for this segment: ToA + ~21.0 ms host overhead = ~51.0 ms

--- RX inter-arrival jitter (cross-check) ---
  rx_iat (L072 hw timestamp_us)    (no samples)
  expected ~= TX inter_cycle_s + 1x ToA jitter

--- Estimated radio-link RTT (bench-tier surrogate) ---
  rtt_p50_estimate_ms = 2 * elapsed_p50 = 0.00 ms
  rtt_p99_estimate_ms = 2 * elapsed_p99 = 0.00 ms
  W4-00(b) target  = 2*toa_p50 + 30 ms slop = 90.00 ms (+/- 5% = 4.50 ms)
  W4-00(b) verdict  : NO_DATA

--- Reconciliation vs LATENCY_BUDGET.md §1 ---
  This bench tier covers ONLY rows #1-#5 of the budget (encode +
  air + demod + AES + IPC). Rows #6-#13 (M4 100 Hz quant, Modbus,
  Opta SSR, coil ramp, spool shift, cylinder pressure) are NOT in
  scope here and add the remaining ~70-160 ms typical / ~580 ms
  worst case to the end-to-end stick->hydraulic figure.

