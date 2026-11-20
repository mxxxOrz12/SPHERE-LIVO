#!/usr/bin/env bash
# Replay an Oxford Spires prefix or the complete bag, keeping the mapper
# alive until it actually writes poses through the requested duration.
set -eo pipefail

repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
: "${BAG_PATH:?Set BAG_PATH to your Oxford Spires ROS1 bag}"
bag="$BAG_PATH"
output="${OUTPUT_DIR:-$repo/results/observatory-quarter-01}"
# Usage: run_oxford_full.sh [name] [rate] [duration|full] [ablation]
# The ablation is "none" or a file name in config/ablation without extension.
name=${1:-oxford_spherical_full}
rate=${2:-0.2}
duration=${3:-full}
ablation=${4:-none}
result="$repo/Log/result/$name.txt"
timing="$repo/Log/result/${name}_timing.csv"
output_result="$output/trajectories/$name.txt"
output_timing="$output/trajectories/${name}_timing.csv"

source /opt/ros/noetic/setup.bash
source "${CATKIN_SETUP:-$repo/../../devel/setup.bash}"
set -u
mkdir -p "$repo/Log/result" "$output/trajectories" "$output/logs" "$output/ros_logs"
export ROS_LOG_DIR="$output/ros_logs"
if [[ -e "$result" || -L "$result" || -e "$timing" || -L "$timing" || -e "$output_result" || -e "$output_timing" ]]; then
  echo "Refusing to overwrite existing trajectory: $result" >&2
  exit 2
fi
ln -s "$output_result" "$result"
ln -s "$output_timing" "$timing"

read -r bag_start bag_end < <(python3 - "$bag" <<'PY'
import rosbag
import sys
with rosbag.Bag(sys.argv[1]) as bag:
    print(bag.get_start_time(), bag.get_end_time())
PY
)
play_args=(-r "$rate")
if [[ "$duration" == full ]]; then
  target=$(awk -v t="$bag_end" 'BEGIN {printf "%.3f", t - 0.5}')
else
  play_args+=(--duration="$duration")
  target=$(awk -v t="$bag_start" -v d="$duration" 'BEGIN {printf "%.3f", t + d - 0.5}')
fi
echo "Bag start=$bag_start, end=$bag_end, duration=$duration; waiting for final pose >= $target"

roscore > "$output/logs/${name}_roscore.log" 2>&1 &
core_pid=$!
mapper_pid=
cleanup() {
  if [[ -n "$mapper_pid" ]]; then
    kill -INT "$mapper_pid" 2>/dev/null || true
    wait "$mapper_pid" 2>/dev/null || true
  fi
  kill -INT "$core_pid" 2>/dev/null || true
  wait "$core_pid" 2>/dev/null || true
}
trap cleanup EXIT

for _ in {1..30}; do
  if rostopic list > /dev/null 2>&1; then break; fi
  sleep 1
done
cd "$repo"
roslaunch fast_livo mapping_sphere_oxford.launch \
  ablation:="$ablation" seq_name:="$name" \
  > "$output/logs/${name}_mapper.log" 2>&1 &
mapper_pid=$!
for _ in {1..30}; do
  if rostopic list | grep -q '^/cloud_registered$'; then break; fi
  sleep 1
done

echo "Playing bag at rate=$rate, duration=$duration"
rosbag play "${play_args[@]}" "$bag" \
  > "$output/logs/${name}_rosbag.log" 2>&1
echo "Bag playback ended; allowing the mapper to drain pending measurements."

for ((i=0; i<120; i++)); do
  if ! kill -0 "$mapper_pid" 2>/dev/null; then
    echo "Mapper exited before reaching the end of the bag" >&2
    exit 3
  fi
  last=0
  count=0
  if [[ -s "$result" ]]; then
    last=$(tail -n 1 "$result" | awk '{print $1}')
    count=$(wc -l < "$result")
  fi
  echo "Drain check: poses=$count, final pose=$last, target=$target"
  if awk -v x="$last" -v y="$target" 'BEGIN {exit !(x >= y)}'; then
    echo "Complete-bag trajectory reached the final timestamp."
    exit 0
  fi
  sleep 10
done
echo "Timed out before the trajectory reached the bag end" >&2
exit 4
