# Dev container for the autonomous ground robot.
# Serves BOTH sides of development from one image:
#   * ROS 2 (Jazzy) node authoring + colcon builds  -> deployed to the Pi
#   * ESP32 firmware compile (PlatformIO) + optional flash
#   * STM32 flash tooling (st-flash / OpenOCD) for the hoverboard mainboard
#
# The container runs on the x86_64 dev machine (WSL2). The robot RUNTIME lives on
# the ARM64 Pi — hardware (serial, camera, GPIO) is not present here. This image
# is for writing, compiling and unit-testing logic, not for talking to real motors.

FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# --- Core dev + firmware toolchain -------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git curl wget vim less \
      python3-pip python3-venv \
      python3-colcon-common-extensions python3-rosdep \
      stlink-tools openocd \
      usbutils udev \
      python3-serial python3-smbus2 \
    && rm -rf /var/lib/apt/lists/*

# --- ROS 2 packages central to the robot (trim if the image gets too big) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-robot-localization ros-jazzy-imu-filter-madgwick \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
      ros-jazzy-diff-drive-controller \
      ros-jazzy-nmea-navsat-driver \
      ros-jazzy-image-transport ros-jazzy-camera-info-manager \
      ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# --- PlatformIO (ESP32 firmware build + flash) -------------------------------
RUN pip3 install --break-system-packages platformio

# --- Serial device access for flashing (when a device is passed through) -----
RUN usermod -aG dialout ubuntu || true

# --- Passwordless sudo for the dev user --------------------------------------
# The base image's 'ubuntu' user is already in the sudo group, but its password
# is locked — so sudo prompts for a password nobody knows and every apt install
# dies. This is what the devcontainers 'common-utils' feature would set up; we
# write it explicitly instead so it is visible here rather than implied.
#
# Scope: a throwaway dev container. Nothing in it runs on the robot — the Pi is
# provisioned from docs/deployment.md, not from this image.
# visudo -c fails the build on a malformed rule, rather than at first use when
# sudo would refuse to run at all.
RUN echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu \
    && visudo -c -f /etc/sudoers.d/ubuntu

USER ubuntu
RUN rosdep update || true
