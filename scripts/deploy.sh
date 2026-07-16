#!/usr/bin/env bash
# Push local commits, then pull + build on the Pi over SSH.
#
# Usage:   PI_HOST=enes@robot.local ./scripts/deploy.sh
#          ./scripts/deploy.sh            # uses defaults below
#
# Assumes the repo is already cloned on the Pi at $REPO_DIR and the colcon
# workspace lives in $REPO_DIR/ros2 (see docs/deployment.md).

set -euo pipefail

PI_HOST="${PI_HOST:-enes@robot.local}"
REPO_DIR="${REPO_DIR:-\$HOME/howerboard}"   # expanded on the Pi, not locally
WS_SUBDIR="${WS_SUBDIR:-ros2}"

echo ">> pushing local commits..."
git push

echo ">> pull + build on ${PI_HOST}..."
ssh "${PI_HOST}" bash -lc "'
  set -euo pipefail
  cd ${REPO_DIR}
  git pull --ff-only
  cd ${WS_SUBDIR}
  source /opt/ros/jazzy/setup.bash
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install
  echo \">> build ok\"
'"

echo ">> done. Restart nodes on the Pi (or the robot.service) to pick up changes."
