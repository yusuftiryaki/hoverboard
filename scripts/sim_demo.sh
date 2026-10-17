#!/usr/bin/env bash
# The whole robot in the simulator, with the web page — for trying missions by
# hand from a browser, no hardware anywhere.
#
#   ./scripts/sim_demo.sh                    then open http://localhost:8080/
#   ./scripts/sim_demo.sh --quiet-gps        GPS without the NEO-6M's wander
#   ./scripts/sim_demo.sh --lat 41.05 --lon 29.01 --site ~/my_site.geojson
#   ./scripts/sim_demo.sh --rock 6,1         an unsurveyed rock (sim_world metres
#                                            east,north of the start) for the bumper
#
# What runs: sim_node (the world, the fake ESP32, IMU, magnetometer, GPS) and
# robot.launch.py exactly as on the Pi — bridge, EKFs, navsat_transform, Nav2,
# battery supervisor, site_map_server, mission_runner, web_ui. The robot starts
# at --lat/--lon facing east. Draw a lawn next to it on the page, save, start a
# mission. Ctrl-C stops everything. The 📷 button on the page shows the
# simulated camera (sim_camera.py: grass, sky, obstacles as grey cylinders —
# for trying the page, nothing more); a bumper press keeps its frame with
# the detection.
#
# The site file persists between runs (default ~/.robot/sim_site.geojson), and
# so do the bumper's detections (~/.robot/sim_detected.json) — delete them for
# a clean start. The docks.yaml station is NOT simulated here: a low battery
# would send the robot to dock at nothing, so the demo pack is 200 Ah with no
# idle draw — it does not run low in a day. (With the stock 4.4 Ah it went
# LOW, then CRITICAL -> SAFE_STOP, after ~3.5 h of an idle demo.)

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Yıldız Parkı, İstanbul — open lawn under trees, so the satellite layer shows
# land. The sim's own default (41.0, 29.0) is in the Bosphorus.
LAT=41.0486
LON=29.0128
SITE="${HOME}/.robot/sim_site.geojson"
DETECTED="${HOME}/.robot/sim_detected.json"
PORT=8080
GPS_ARGS=()
ROCK_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --lat) LAT="$2"; shift 2 ;;
    --lon) LON="$2"; shift 2 ;;
    --site) SITE="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --quiet-gps) GPS_ARGS=(-p gps_wander_sigma:=0.0 -p gps_noise_sigma:=0.02); shift ;;
    --rock)
      # Doubles for ROS: "6" would arrive as an int and DOUBLE_ARRAY rejects it.
      IFS=, read -r RX RY <<< "$2"
      ROCK_ARGS=(-p "unsurveyed_obstacle_centers:=[$(printf '%.2f' "${RX}"), $(printf '%.2f' "${RY}")]"
                 -p "unsurveyed_obstacle_radii:=[0.3]")
      shift 2 ;;
    -h|--help) sed -n '2,22p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $1 (see --help)" >&2; exit 2 ;;
  esac
done

mkdir -p "$(dirname "${SITE}")" "$(dirname "${DETECTED}")"

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO}/ros2/install/setup.bash" 2>/dev/null || {
  echo "ros2/install yok — önce: cd ros2 && colcon build --symlink-install" >&2; exit 1; }
set -u

if pgrep -f "lib/robot_sim/sim_node|robot_bringup robot.launch.py" >/dev/null; then
  echo "Bir simülasyon/robot yığını zaten çalışıyor (pgrep). Önce onu kapat." >&2
  exit 1
fi

cleanup() {
  echo; echo ">> kapatılıyor"
  kill -INT -- "-${SIM_PID}" "-${STACK_PID}" 2>/dev/null || true
  sleep 3
  kill -KILL -- "-${SIM_PID}" "-${STACK_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo ">> simülasyon: başlangıç ${LAT}, ${LON}"
setsid ros2 run robot_sim sim_node --ros-args \
  -p datum_lat:="${LAT}" -p datum_lon:="${LON}" \
  -p publish_obstacle_map:=false -p bumper_from_obstacles:=true \
  -p battery_capacity_ah:=200.0 -p battery_idle_current_a:=0.0 \
  -p camera_rate_hz:=5.0 \
  "${GPS_ARGS[@]}" "${ROCK_ARGS[@]}" &
SIM_PID=$!
sleep 3

echo ">> robot yığını (robot.launch.py, Nav2 dahil)"
setsid ros2 launch robot_bringup robot.launch.py esp32_port:=/tmp/fake_esp32 \
  use_localization:=true use_gps:=true use_imu:=false use_mag:=false \
  use_imu_filter:=true use_nav2:=true \
  site_file:="${SITE}" detected_file:="${DETECTED}" web_port:="${PORT}" &
STACK_PID=$!

echo
echo ">> Web arayüzü: http://localhost:${PORT}/   (Nav2'nin kalkması ~30 sn)"
echo ">> Saha dosyası: ${SITE}"
echo ">> Durdurmak için Ctrl-C"
wait "${STACK_PID}"
