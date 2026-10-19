#!/usr/bin/env bash
# E3 / E2c route: 2 laps W1 -> W2 -> W3 -> W5 (W4 skipped: rack-wall strip traps Nav2).
# Usage: bash e3_route.sh <run_name>
DIR="/mnt/c/Users/Har Zi Ao/Documents/ICTE4001_SLAM"
RUN="${1:-e3_run}"
export E4_CSV="$DIR/${RUN}_waypoints.csv"
for LAP in 1 2; do
  bash "$DIR/e4_goal.sh" L${LAP}_W1 5.9 6.1 -3.13
  bash "$DIR/e4_goal.sh" L${LAP}_W2 -5.5 6.0 -1.64
  bash "$DIR/e4_goal.sh" L${LAP}_W3 -6.3 -6.3 0.93
  bash "$DIR/e4_goal.sh" L${LAP}_W5 -1.0 0.8 0.65
done
echo; echo "=== $RUN route finished. Results: ${RUN}_waypoints.csv ==="
