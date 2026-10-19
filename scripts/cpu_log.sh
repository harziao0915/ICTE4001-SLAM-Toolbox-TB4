#!/usr/bin/env bash
# Log CPU % and memory of the SLAM Toolbox node once per second until Ctrl+C.
# Usage: bash cpu_log.sh <run_name>
DIR="/mnt/c/Users/Har Zi Ao/Documents/ICTE4001_SLAM"
RUN="${1:-run}"; OUT="$DIR/${RUN}_slam_cpu.csv"
PID=$(pgrep -f "slam_toolbox_node" | head -1)
[ -z "$PID" ] && { echo "SLAM Toolbox not running - start SLAM first"; exit 1; }
echo "Logging PID $PID (slam_toolbox) to ${RUN}_slam_cpu.csv  - press Ctrl+C at the end of the run"
echo "wall_time,cpu_percent,rss_MB" > "$OUT"
top -b -d 1 -p "$PID" | awk -v out="$OUT" '$1==pid {cmd="date +%T"; cmd | getline t; close(cmd); r=$6; if(r ~ /g$/){r=r*1024}else if(r ~ /m$/){r=r+0}else{r=r/1024}; printf "%s,%s,%.1f\n", t, $9, r >> out; fflush(out); printf "\r  CPU %5s %%   RSS %7.1f MB ", $9, r}' pid="$PID"
