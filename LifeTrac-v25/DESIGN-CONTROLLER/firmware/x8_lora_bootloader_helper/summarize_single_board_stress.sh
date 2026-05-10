#!/bin/bash
# summarize_single_board_stress.sh — build a markdown report from a stress run.
#
# Usage:
#   bash /tmp/lifetrac_p0c/summarize_single_board_stress.sh [run_dir]
#
# Defaults:
#   run_dir = latest /tmp/lifetrac_p0c/stress_runs/<timestamp>
#
# Output:
#   <run_dir>/report.md

set -u

ROOT=/tmp/lifetrac_p0c/stress_runs
RUN_DIR=${1:-}

if [ -z "$RUN_DIR" ]; then
  if [ ! -d "$ROOT" ]; then
    echo "ERROR: stress run root missing: $ROOT"
    exit 2
  fi

  RUN_DIR=$(ls -1dt "$ROOT"/* 2>/dev/null | head -1)
fi

if [ -z "$RUN_DIR" ] || [ ! -d "$RUN_DIR" ]; then
  echo "ERROR: run directory not found: $RUN_DIR"
  exit 2
fi

TSV=$RUN_DIR/results.tsv
SUMMARY_TXT=$RUN_DIR/summary.txt
REPORT_MD=$RUN_DIR/report.md

if [ ! -f "$TSV" ]; then
  echo "ERROR: missing results file: $TSV"
  exit 2
fi

TOTAL=$(awk 'NR>1 {n++} END {print n+0}' "$TSV")
PASS=$(awk -F '\t' 'NR>1 && $4==0 {n++} END {print n+0}' "$TSV")
FAIL=$(awk -F '\t' 'NR>1 && $4!=0 {n++} END {print n+0}' "$TSV")
BANNER_HITS=$(awk -F '\t' 'NR>1 && $5==1 {n++} END {print n+0}' "$TSV")
TICK_HITS=$(awk -F '\t' 'NR>1 && $6==1 {n++} END {print n+0}' "$TSV")
REBOOT_HITS=$(awk -F '\t' 'NR>1 && $8==1 {n++} END {print n+0}' "$TSV")

AVG_RX_BYTES=$(awk -F '\t' 'NR>1 {s+=$7; n++} END {if (n>0) printf "%.1f", s/n; else print "0.0"}' "$TSV")
MIN_RX_BYTES=$(awk -F '\t' 'NR>1 {if (min=="" || $7<min) min=$7} END {if (min=="") print 0; else print min}' "$TSV")
MAX_RX_BYTES=$(awk -F '\t' 'NR>1 {if ($7>max) max=$7} END {print max+0}' "$TSV")

AVG_SEC=$(awk -F '\t' 'NR>1 {s+=($3-$2); n++} END {if (n>0) printf "%.1f", s/n; else print "0.0"}' "$TSV")
MIN_SEC=$(awk -F '\t' 'NR>1 {d=$3-$2; if (min=="" || d<min) min=d} END {if (min=="") print 0; else print min}' "$TSV")
MAX_SEC=$(awk -F '\t' 'NR>1 {d=$3-$2; if (d>max) max=d} END {print max+0}' "$TSV")

if [ "$TOTAL" -gt 0 ]; then
  PASS_PCT=$(awk -v p="$PASS" -v t="$TOTAL" 'BEGIN {printf "%.1f", (100.0*p)/t}')
  BANNER_PCT=$(awk -v p="$BANNER_HITS" -v t="$TOTAL" 'BEGIN {printf "%.1f", (100.0*p)/t}')
  TICK_PCT=$(awk -v p="$TICK_HITS" -v t="$TOTAL" 'BEGIN {printf "%.1f", (100.0*p)/t}')
else
  PASS_PCT="0.0"
  BANNER_PCT="0.0"
  TICK_PCT="0.0"
fi

if awk -v p="$PASS_PCT" -v b="$BANNER_PCT" 'BEGIN {exit !((p+0)>=95.0 && (b+0)>=95.0)}'; then
  RECOMMENDATION="READY_FOR_NEXT_STAGE"
else
  RECOMMENDATION="STAY_IN_SINGLE_BOARD_HARDENING"
fi

FIRST_EPOCH=$(awk -F '\t' 'NR==2 {print $2; exit}' "$TSV")
LAST_EPOCH=$(awk -F '\t' 'NR>1 {last=$3} END {print last+0}' "$TSV")

{
  echo "# Method G Single-Board Stress Report"
  echo
  echo "- Generated: $(date -u '+%Y-%m-%d %H:%M:%S UTC')"
  echo "- Run directory: $RUN_DIR"
  echo "- Source TSV: $TSV"
  if [ -f "$SUMMARY_TXT" ]; then
    echo "- Summary log: $SUMMARY_TXT"
  fi
  echo
  echo "## Results"
  echo
  echo "| Metric | Value |"
  echo "|---|---:|"
  echo "| Total cycles | $TOTAL |"
  echo "| Pass | $PASS |"
  echo "| Fail | $FAIL |"
  echo "| Pass rate | $PASS_PCT% |"
  echo "| Banner hit count | $BANNER_HITS |"
  echo "| Banner hit rate | $BANNER_PCT% |"
  echo "| Tick hit count | $TICK_HITS |"
  echo "| Tick hit rate | $TICK_PCT% |"
  echo "| Auto-reboot heuristic hits | $REBOOT_HITS |"
  echo "| Avg cycle seconds | $AVG_SEC |"
  echo "| Min cycle seconds | $MIN_SEC |"
  echo "| Max cycle seconds | $MAX_SEC |"
  echo "| Avg rx bytes | $AVG_RX_BYTES |"
  echo "| Min rx bytes | $MIN_RX_BYTES |"
  echo "| Max rx bytes | $MAX_RX_BYTES |"
  echo
  echo "## Recommendation"
  echo
  echo "- $RECOMMENDATION"
  echo
  echo "## Time Window"
  echo
  echo "- First cycle start epoch: $FIRST_EPOCH"
  echo "- Last cycle end epoch: $LAST_EPOCH"
  echo
  echo "## Notes"
  echo
  echo "- Recommendation rule matches the one-board gate currently used by the stress wrapper: pass >= 95% and banner-hit >= 95%."
  echo "- Auto-reboot detection is text heuristic only."
} > "$REPORT_MD"

echo "report=$REPORT_MD"
exit 0
