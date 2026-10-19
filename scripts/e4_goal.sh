#!/usr/bin/env bash
# E4 helper v2 — send ONE Nav2 goal (map frame), wait for the result, log it.
# v2: no --feedback printing (it ran ~100 msg/s and stole CPU from the slow sim);
#     sim time measured from /clock; recoveries sampled every ~30 s;
#     big raw logs kept in ~/e4_logs (Linux disk), only the CSV goes to Windows.
# Usage:  bash e4_goal.sh <goal_id> <x> <y> <yaw_rad>
DIR="/mnt/c/Users/Har Zi Ao/Documents/ICTE4001_SLAM"
LOGD="$HOME/e4_logs"; mkdir -p "$LOGD"
ID="$1"; X="$2"; Y="$3"; YAW="${4:-0}"
if [ -z "$ID" ] || [ -z "$X" ] || [ -z "$Y" ]; then
  echo "Usage: bash e4_goal.sh <goal_id> <x> <y> <yaw_rad>"; exit 1; fi
RAW="$LOGD/e4_raw_${ID}_$(date +%H%M%S).txt"
CSV="${E4_CSV:-$DIR/e4_results.csv}"
[ -f "$CSV" ] || echo "goal,x_goal,y_goal,yaw_goal,status,nav_time_s,dist_remaining_m,recoveries,wall_time_s,slam_x,slam_y,gt_x,gt_y,timestamp" > "$CSV"

simsec() { timeout 20 ros2 topic echo /clock --once --field clock 2>/dev/null | awk '/sec:/{if($1=="sec:")s=$2; else n=$2} END{printf "%.1f", s+n/1e9}'; }
QZ=$(python3 -c "import math;print(round(math.sin($YAW/2),4))")
QW=$(python3 -c "import math;print(round(math.cos($YAW/2),4))")

echo "=== $ID -> goal (x=$X, y=$Y, yaw=$YAW) in map frame. Slow sim: can take a while..."
S0=$(simsec); T0=$(date +%s)
RECF="$LOGD/.rec_$$"; echo 0 > "$RECF"
( while true; do
    sleep 30
    R=$(timeout 10 ros2 topic echo /navigate_to_pose/_action/feedback --once --field feedback.number_of_recoveries 2>/dev/null | grep -E "^[0-9]+$" | head -1)
    [ -n "$R" ] && echo "$R" > "$RECF"
    echo "   ...still driving: $(( $(date +%s)-T0 )) s wall, recoveries so far: $(cat "$RECF")"
  done ) &
MON=$!
trap 'echo; echo ">>> Ctrl+C: cancelling goal and logging it..."' INT
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}}" \
  > "$RAW" 2>&1
kill $MON 2>/dev/null; wait $MON 2>/dev/null
REC=$(cat "$RECF"); rm -f "$RECF"
trap - INT
S1=$(simsec); T1=$(date +%s)
STATUS=$(grep -o "finished with status: [A-Z_]*" "$RAW" | tail -1 | awk '{print $NF}')
[ -z "$STATUS" ] && STATUS="CANCELLED"
NAVT=$(python3 -c "print(round($S1-$S0,1))" 2>/dev/null)

TF=$(timeout 10 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -m1 "Translation")
SX=$(echo "$TF" | sed 's/.*\[\(.*\)\].*/\1/' | cut -d, -f1 | tr -d ' ')
SY=$(echo "$TF" | sed 's/.*\[\(.*\)\].*/\1/' | cut -d, -f2 | tr -d ' ')
DIST=$(python3 -c "import math;print(round(math.hypot($X-($SX),$Y-($SY)),3))" 2>/dev/null)
GTT=$(ros2 topic type /sim_ground_truth_pose 2>/dev/null | head -1)
GT=$(timeout 15 ros2 topic echo /sim_ground_truth_pose $GTT --once 2>/dev/null)
echo "$GT" >> "$RAW"
GX=$(echo "$GT" | awk '/position:/{f=1;next} f&&$1=="x:"{print $2;exit}')
GY=$(echo "$GT" | awk '/position:/{f=1;next} f&&$1=="y:"{print $2;exit}')

echo "$ID,$X,$Y,$YAW,$STATUS,${NAVT:-},${DIST:-},${REC:-0},$((T1-T0)),${SX:-},${SY:-},${GX:-},${GY:-},$(date '+%F %T')" >> "$CSV"
echo
echo "=========== $ID RESULT ==========="
echo " status            : $STATUS"
echo " nav time (sim)    : ${NAVT:-?} s      wall time: $((T1-T0)) s"
echo " error to goal     : ${DIST:-?} m (SLAM pose vs goal)   recoveries: ${REC:-0}"
echo " SLAM pose (map)   : x=${SX:-?}  y=${SY:-?}"
echo " ground truth      : x=${GX:-?}  y=${GY:-?}   (Gazebo world frame)"
echo " saved -> e4_results.csv  (raw log: $RAW)"
