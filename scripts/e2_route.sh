#!/usr/bin/env bash
# E2 route: 2 laps of a big loop round the depot via 5 Nav2 waypoints (same route for loop-closure ON and OFF).
# Usage: bash e2_route.sh <run_name>     e.g.  bash e2_route.sh e2_loop_on
DIR="/mnt/c/Users/Har Zi Ao/Documents/ICTE4001_SLAM"
RUN="${1:-e2_run}"
export E4_CSV="$DIR/${RUN}_waypoints.csv"
for LAP in 1 2; do
  bash "$DIR/e4_goal.sh" L${LAP}_W1 5.9 6.1 -3.13
  bash "$DIR/e4_goal.sh" L${LAP}_W2 -5.5 6.0 -1.64
  bash "$DIR/e4_goal.sh" L${LAP}_W3 -6.3 -6.3 -0.01
  bash "$DIR/e4_goal.sh" L${LAP}_W4 1.8 -6.4 1.94
  bash "$DIR/e4_goal.sh" L${LAP}_W5 -1.0 0.8 0.65
done
echo; echo "=== $RUN route finished. Results: ${RUN}_waypoints.csv ==="
