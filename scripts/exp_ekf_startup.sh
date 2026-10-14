#!/usr/bin/env bash
# EXPERIMENT (exp/ekf-startup branch only, not for main), round 4.
# Rounds 1-3 never reproduced the CI divergence (0/52 stacks) while ci.sh's own
# stack/nav2 stages diverged almost every stack. So call ci.sh UNMODIFIED, then
# again with only ekf_local's debug flag flipped by a text edit (no YAML
# round-trip), and read every stack's startup from the bag ci.sh records.
set -uo pipefail
REPO=/workspaces/hoverboard
OUT=$REPO/ros2/log/ci/exp
mkdir -p "$OUT"
CFG=$REPO/ros2/src/robot_bringup/config/ekf.yaml
cp "$CFG" "$OUT/ekf.orig.yaml"
export CI_BAG=1

verdict() {  # verdict <bag>
  (set +u; source /opt/ros/jazzy/setup.bash; source "$REPO/ros2/install/setup.bash"
   python3 "$REPO/scripts/exp_bag_startup.py" "$1" 2>&1 | awk '/########/{n=0; next} /EKF/{n++; if(n==2) print ($0 ~ /e\+1[0-9][0-9]|e\+2[0-9][0-9]/ ? "X" : ".")}' | tr -d '\n')
}

run() {  # run <stage> <label> [debug]
  cp "$OUT/ekf.orig.yaml" "$CFG"
  if [[ ${3:-} == debug ]]; then
    # first `debug: false` in the file is ekf_local's
    sed -i "0,/^    debug: false$/s||    debug: true\n    debug_out_file: $OUT/ekf_debug_$2.txt|" "$CFG"
    grep -n "debug" "$CFG" | head -3
  fi
  "$REPO/scripts/ci.sh" "$1" > "$OUT/ci-$2.log" 2>&1
  local rc=$?
  mv "$REPO/ros2/log/ci/bag-$1" "$OUT/bag-$2" 2>/dev/null
  echo "$2 ci.sh $1 exit=$rc stacks[$(verdict "$OUT/bag-$2")] $(grep -E '[0-9]+ (passed|failed)' "$OUT/ci-$2.log" | tail -1)"
  [[ -f "$OUT/ekf_debug_$2.txt" ]] && truncate -s '<60M' "$OUT/ekf_debug_$2.txt"
  cp "$OUT/ekf.orig.yaml" "$CFG"
}

{
  run stack stack-plain
  run stack stack-debug debug
  run nav2 nav2-plain
  run nav2 nav2-debug debug
} | tee "$OUT/results.txt"
