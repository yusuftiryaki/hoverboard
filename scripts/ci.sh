#!/usr/bin/env bash
# Everything CI runs, in one place. GitHub Actions calls this script and does
# nothing else, so the same command gives the same answer in the dev container.
#
# Usage:   ./scripts/ci.sh <stage>...
#
#   unit      every package's tests with NO ROS on the path        (seconds)
#   firmware  pio run                                              (~1 min)
#   stack     colcon build + every test except the four below      (~4 min)
#   nav2      colcon build + the Nav2 stacks (map pinned, and on GPS) (~7 min)
#   gazebo    colcon build + the Gazebo physics worlds             (~5 min)
#   dock      colcon build + battery -> return -> dock stacks      (~5 min)
#   sweep     colcon build + docking from every side, one stack    (~12 min)
#   mission   colcon build + coverage missions, one with a charge  (~20 min)
#   all       the lot, in that order
#
# stack + nav2 + gazebo + dock + sweep + mission together are the whole suite: `stack` is defined as
# "everything that is not in HEAVY", so a new test file lands there by default
# and cannot fall between the jobs.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${REPO}/ros2"
REPORTS="${WS}/log/ci"          # ros2/log/ is already gitignored

NAV2_TESTS=(src/robot_sim/test/test_nav2.py src/robot_sim/test/test_site_nav2.py)
GAZEBO_TESTS=(src/robot_sim/test/test_gazebo_physics.py
              src/robot_sim/test/test_gazebo_localization.py)
DOCK_TESTS=(src/robot_sim/test/test_docking.py)
SWEEP_TESTS=(src/robot_sim/test/test_docking_sweep.py)
MISSION_TESTS=(src/robot_sim/test/test_mission_coverage.py src/robot_sim/test/test_mission_dock.py)
HEAVY=("${NAV2_TESTS[@]}" "${GAZEBO_TESTS[@]}" "${DOCK_TESTS[@]}" "${SWEEP_TESTS[@]}"
       "${MISSION_TESTS[@]}")

packages_with_tests() {
  local dir
  for dir in "${WS}"/src/*/test; do
    basename "$(dirname "${dir}")"
  done
}

# --- unit: the ROS-free claim ------------------------------------------------
# protocol.py, world.py and the register-level drivers are ROS-free on purpose,
# and their tests are meant to pass with the ROS ones skipping cleanly. A
# module-level ROS import in a test file breaks that as a COLLECTION ERROR for
# the whole directory, and nothing notices on a machine where ROS is always
# sourced. So ROS is taken off the path here rather than assumed absent.
# Skips are expected in this stage — they are the ROS tests.
stage_unit() {
  local pkg
  for pkg in $(packages_with_tests); do
    echo ">> unit: ${pkg}"
    (cd "${WS}/src/${pkg}" &&
       env -u PYTHONPATH -u AMENT_PREFIX_PATH \
         python3 -m pytest test -q -rs -p no:cacheprovider)
  done
}

stage_firmware() {
  echo ">> firmware: pio run"
  (cd "${REPO}/firmware/esp32_bridge" && pio run)
}

# --- the ROS stages ----------------------------------------------------------
BUILT=0
build_workspace() {
  [[ "${BUILT}" == 1 ]] && return
  echo ">> colcon build"
  cd "${WS}"
  # The ROS setup scripts read variables they never set.
  set +u
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install
  source install/setup.bash
  set -u
  mkdir -p "${REPORTS}"
  BUILT=1
}

# --- flight recorder (CI_BAG=1) ----------------------------------------------
# The stacks run with stdout discarded, and an EKF that diverges says nothing in
# its log: the first CI run lost ekf_local to 8.6e147 m and left only "Failed
# to meet update rate" behind. So in CI every ROS stage records what the filter
# was fed and what it said, and the workflow uploads it when the stage fails.
RECORDED_TOPICS=(/odom /imu/data_raw /imu/data /odometry/filtered/local /cmd_vel_nav
                 /odometry/filtered/global /ground_truth /gazebo/ground_truth
                 /cmd_vel /cmd_vel_teleop /cmd_vel_nav_gated /diagnostics
                 /battery /battery_raw /sim/battery_current /nav_lock
                 /detected_dock_pose /dock_contact_truth /battery_supervisor/state /tf /tf_static /bumper /collision_truth
                 /mission_runner/status /obstacle_map)
RECORDER_PID=
start_recorder() {
  [[ -n "${CI_BAG:-}" ]] || return 0
  stop_recorder
  local out="${REPORTS}/bag-$1"
  rm -rf "${out}"
  # Its own session, so stopping it cannot signal the stacks, and its command
  # line matches none of the conftest's stray-node patterns.
  setsid ros2 bag record -s mcap -o "${out}" \
    --topics "${RECORDED_TOPICS[@]}" > "${out}.log" 2>&1 &
  RECORDER_PID=$!
  trap stop_recorder EXIT
}
stop_recorder() {
  [[ -n "${RECORDER_PID}" ]] || return 0
  # ⚠️ SIGTERM, NOT SIGINT. A job started with `&` from a non-interactive
  # shell has SIGINT IGNORED, and Python keeps an inherited ignore — so the
  # recorder never saw the INT and `wait` hung the whole stage after its tests
  # had passed. TERM finalises the bag just as cleanly; KILL is the backstop so
  # a diagnostic can never be what hangs CI.
  kill -TERM -- "-${RECORDER_PID}" 2>/dev/null || true
  local tries
  for tries in $(seq 30); do
    kill -0 "${RECORDER_PID}" 2>/dev/null || break
    sleep 0.5
  done
  kill -KILL -- "-${RECORDER_PID}" 2>/dev/null || true
  wait "${RECORDER_PID}" 2>/dev/null || true
  RECORDER_PID=
}

# run_tests <report name> <pytest args...>
#
# ⚠️ A SKIP FAILS THE STAGE. The stack tests skip when the workspace is not
# sourced and the Gazebo ones skip when `gz` is missing — which is the right
# thing at a desk and exactly wrong here: "10 passed, 2 skipped" is green, and
# the two that skipped are the ones that would have caught the bug. In CI the
# environment is supposed to be complete, so a skip means it is not.
run_tests() {
  local report="${REPORTS}/$1.xml"
  shift
  # faulthandler_timeout: a test running longer than this dumps every
  # thread's stack into the log. A nav2 run once hung for three hours locally
  # and the only record of why was lost; CI's job timeout would have killed it
  # just as silently.
  python3 -m pytest "$@" -q -rs -p no:cacheprovider --junitxml="${report}" \
    -o faulthandler_timeout=600
  python3 - "${report}" <<'EOF'
import sys
import xml.etree.ElementTree as ET

suite = ET.parse(sys.argv[1]).getroot().find("testsuite")
tests, skipped = int(suite.get("tests")), int(suite.get("skipped"))
if skipped or not tests:
    sys.exit(f"!! {skipped} of {tests} tests skipped in {sys.argv[1]} — in CI a "
             "skip is a hole in the environment, not a pass. See the SKIPPED "
             "lines above.")
EOF
}

stage_stack() {
  build_workspace
  start_recorder stack
  # ⚠️ The test FILES are listed, not the directory with --ignore. launch_testing
  # 3.4.12 (a pytest plugin every ROS install loads) adds a tryfirst
  # pytest_ignore_collect that returns False instead of None for every path —
  # and that hook is firstresult, so pytest's own --ignore never runs. The
  # heavy files then ran twice, inside this stage too, and nothing failed: the
  # stage just took 566 s instead of 130.
  local pkg file files
  for pkg in $(packages_with_tests); do
    files=()
    for file in "src/${pkg}/test/"test_*.py; do
      [[ " ${HEAVY[*]} " == *" ${file} "* ]] || files+=("${file}")
    done
    echo ">> stack: ${pkg}"
    run_tests "stack-${pkg}" "${files[@]}"
  done
}

stage_nav2() {
  build_workspace
  start_recorder nav2
  echo ">> nav2"
  run_tests nav2 "${NAV2_TESTS[@]}"
}

stage_gazebo() {
  build_workspace
  start_recorder gazebo
  echo ">> gazebo"
  run_tests gazebo "${GAZEBO_TESTS[@]}"
}

stage_dock() {
  build_workspace
  start_recorder dock
  echo ">> dock"
  run_tests dock "${DOCK_TESTS[@]}"
}

stage_mission() {
  build_workspace
  start_recorder mission
  echo ">> mission"
  # -s: the coverage numbers are the result worth reading, pass or fail.
  run_tests mission -s "${MISSION_TESTS[@]}"
}

stage_sweep() {
  build_workspace
  start_recorder sweep
  echo ">> sweep"
  # -s: the sweep's table is the result worth reading, pass or fail.
  run_tests sweep -s "${SWEEP_TESTS[@]}"
}

[[ $# -gt 0 ]] || { sed -n '2,18p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 2; }

for stage in "$@"; do
  case "${stage}" in
    unit|firmware|stack|nav2|gazebo|dock|sweep|mission) "stage_${stage}" ;;
    all) stage_unit; stage_firmware; stage_stack; stage_nav2; stage_gazebo; stage_dock
         stage_sweep; stage_mission ;;
    *) echo "unknown stage: ${stage}" >&2; exit 2 ;;
  esac
done

echo ">> ok: $*"
