#!/usr/bin/env bash
# EXPERIMENT (exp/ekf-startup branch only, not for main): does ekf_local blow up
# at startup on the CI runner, and which input path does it?
#   P  ekf.yaml as is (plain control: debug output changes timing)
#   A  ekf.yaml as is, robot_localization debug on
#   B  imu0 ax NOT fused
#   C  imu0 ax fused, remove_gravitational_acceleration off
#   D  ekf.yaml as is, Fast DDS restricted to UDPv4 (no shared memory)
set -uo pipefail
REPO=/workspaces/hoverboard
WS=$REPO/ros2
OUT=$WS/log/ci/exp
TRIALS=${TRIALS:-3}
mkdir -p "$OUT"
cd "$WS"
set +u; source /opt/ros/jazzy/setup.bash; colcon build --symlink-install > "$OUT/build.log" 2>&1; source install/setup.bash; set -u
CFG=src/robot_bringup/config/ekf.yaml
cp "$CFG" "$OUT/ekf.orig.yaml"

variant() {  # variant <name> <trial>
  python3 - "$OUT/ekf.orig.yaml" "$CFG" "$1" "$2" "$OUT" <<'PY'
import sys, yaml
src, dst, name, trial, out = sys.argv[1:]
d = yaml.safe_load(open(src))
p = d["ekf_local"]["ros__parameters"]
if name == "A":
    p["debug"] = True
    p["debug_out_file"] = f"{out}/ekf_debug_A{trial}.txt"
elif name == "B":
    cfg = list(p["imu0_config"]); cfg[12] = False; p["imu0_config"] = cfg
elif name == "C":
    p["imu0_remove_gravitational_acceleration"] = False
yaml.safe_dump(d, open(dst, "w"), sort_keys=False)
PY
}

measure() {  # print max |x| and |vx| of /odometry/filtered/local over 2 s
  timeout 20 python3 - <<'PY'
import math, time, rclpy
from nav_msgs.msg import Odometry
rclpy.init(); n = rclpy.create_node("exp_probe"); got = []
n.create_subscription(Odometry, "/odometry/filtered/local", got.append, 50)
end = time.monotonic() + 2.0
while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.1)
if not got: print("NO_EKF_OUTPUT"); raise SystemExit
mx = max(abs(m.pose.pose.position.x) for m in got); mv = max(abs(m.twist.twist.linear.x) for m in got)
print(("DIVERGED" if (mx > 10 or not math.isfinite(mx)) else "ok"), f"max|x|={mx:.3e} max|vx|={mv:.3e} n={len(got)}")
PY
}

# Round 3: in CI the FIRST stack of a pytest session was healthy and the 2nd-4th
# diverged at startup; single-stack sessions never did. So run the whole file
# (4 stacks in one session) and read every stack's startup from a bag.
FILE=src/robot_sim/test/test_localization.py
run_file() {  # run_file <label>
  local bag="$OUT/bag-$1"
  setsid ros2 bag record -s mcap -o "$bag" --topics /odometry/filtered/local /odom /imu/data_raw /tf_static > "$bag.log" 2>&1 & local rec=$!
  sleep 2
  local r; r=$(python3 -m pytest "$FILE" -q -p no:cacheprovider 2>&1 | grep -E "passed|failed" | tail -1)
  kill -TERM -- -"$rec" 2>/dev/null; wait "$rec" 2>/dev/null
  local starts; starts=$(python3 "$REPO/scripts/exp_bag_startup.py" "$bag" 2>&1 | awk '/########/{n=0; next} /EKF/{n++; if(n==2) print ($0 ~ /e\+1[0-9][0-9]|e\+2[0-9][0-9]/ ? "X" : ".")}' | tr -d '\n')
  echo "$1 stacks[$starts] (X = diverged at startup) | pytest: $r"
}
{
  variant P 1; unset FASTDDS_BUILTIN_TRANSPORTS; run_file P1
  variant A 1; run_file A1; truncate -s "<40M" "$OUT/ekf_debug_A1.txt"
  variant A 2; run_file A2; truncate -s "<40M" "$OUT/ekf_debug_A2.txt"
  variant P 2; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; run_file D1; unset FASTDDS_BUILTIN_TRANSPORTS
} | tee "$OUT/results.txt"
cp "$OUT/ekf.orig.yaml" "$CFG"
