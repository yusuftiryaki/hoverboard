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
TRIALS=${TRIALS:-6}
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

for name in P A B C D; do
  if [[ $name == D ]]; then export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; else unset FASTDDS_BUILTIN_TRANSPORTS; fi
  for i in $(seq "$TRIALS"); do
    variant "$name" "$i"
    setsid ros2 run robot_sim sim_node > "$OUT/sim_${name}${i}.log" 2>&1 & SIM=$!
    sleep 3
    setsid ros2 launch robot_bringup robot.launch.py esp32_port:=/tmp/fake_esp32 \
      use_localization:=true use_imu:=false use_gps:=false > "$OUT/launch_${name}${i}.log" 2>&1 & STK=$!
    sleep 6
    echo "$name$i $(measure) shm=$(df --output=pcent /dev/shm | tail -1 | tr -d " ")"
    kill -KILL -- -"$STK" -"$SIM" 2>/dev/null
    wait "$STK" "$SIM" 2>/dev/null
    for _ in $(seq 40); do pgrep -f "ekf_nod[e]|robot_sim/sim_nod[e]|hoverboard_bridge/hoverboard_bridg[e]" >/dev/null || break; sleep 0.25; done
  done
done | tee "$OUT/results.txt"
cp "$OUT/ekf.orig.yaml" "$CFG"
echo "== summary"; for name in P A B C D; do echo "$name: $(grep -c "^$name[0-9]* DIVERGED" "$OUT/results.txt")/$TRIALS diverged"; done | tee -a "$OUT/results.txt"
