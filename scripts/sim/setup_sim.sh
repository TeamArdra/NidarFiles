#!/usr/bin/env bash
# One-time setup of the Main Mission (Sim) toolchain on any Ubuntu 24.04
# laptop (or WSL2 Ubuntu-24.04). Needs internet and ~10 GB of disk; the
# ArduPilot build takes 10-30 minutes. Safe to run again: finished steps are
# skipped.
#
# Installs:
#   ROS 2 Jazzy (ros-base) + MAVROS + Cartographer + ros_gz bridges
#   Gazebo Harmonic (simulator)
#   ArduPilot ArduCopter SITL, tag Copter-4.6.3 (same firmware as the drone)
#   ardupilot_gazebo plugin (connects SITL to Gazebo)
#   socat (stands in for the two MicroLR900 radios, see start_sim.sh)
# and builds onboard-autonomy into ~/nidar_sim_ws.
#
# Usage:  scripts/sim/setup_sim.sh          (as your normal user, not sudo)
set -euo pipefail
[ "$(id -u)" -eq 0 ] && { echo "ERROR: run as your normal user, not with sudo." >&2; exit 1; }

NIDAR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SIM_HOME="${NIDAR_SIM_HOME:-$HOME/nidar_sim}"
OVERLAY_WS="${NIDAR_SIM_WS:-$HOME/nidar_sim_ws}"
ARDUPILOT_TAG="${NIDAR_ARDUPILOT_TAG:-Copter-4.6.3}"
say() { echo "== $*"; }

. /etc/os-release
[ "${VERSION_ID:-}" = "24.04" ] || { echo "ERROR: needs Ubuntu 24.04 (found ${PRETTY_NAME:-unknown}): ROS 2 Jazzy + Gazebo Harmonic." >&2; exit 1; }

export DEBIAN_FRONTEND=noninteractive
sudo apt-get update -qq
sudo apt-get install -y -qq curl gnupg lsb-release software-properties-common git socat \
  python3-pip python3-serial python3-venv build-essential cmake

# -- ROS 2 Jazzy ------------------------------------------------------------------
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  say "ROS 2 Jazzy apt source"
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt-get update -qq
fi
# -- Gazebo Harmonic --------------------------------------------------------------
if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
  say "Gazebo Harmonic apt source"
  sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
  sudo apt-get update -qq
fi
say "ROS 2 Jazzy + Gazebo Harmonic packages"
sudo apt-get install -y -qq \
  ros-jazzy-ros-base ros-dev-tools python3-colcon-common-extensions \
  ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs \
  ros-jazzy-cartographer-ros ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
  ros-jazzy-ros-gz-bridge ros-jazzy-geographic-msgs \
  gz-harmonic libgz-sim8-dev rapidjson-dev libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
# MAVROS needs the GeographicLib geoid/magnetic datasets once.
if [ ! -d /usr/share/GeographicLib/geoids ]; then
  say "MAVROS GeographicLib datasets"
  sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
fi

mkdir -p "$SIM_HOME"

# -- ArduPilot SITL (ArduCopter, same version as the drone) ----------------------------
if [ ! -x "$SIM_HOME/ardupilot/build/sitl/bin/arducopter" ]; then
  say "ArduPilot $ARDUPILOT_TAG (clone + prerequisites + SITL build -- the slow part)"
  if [ ! -d "$SIM_HOME/ardupilot/.git" ]; then
    git clone --depth 1 --branch "$ARDUPILOT_TAG" --recurse-submodules --shallow-submodules \
      https://github.com/ArduPilot/ardupilot.git "$SIM_HOME/ardupilot"
  fi
  cd "$SIM_HOME/ardupilot"
  # finishes submodules of an interrupted clone (no-op on a complete one)
  git submodule sync --recursive
  git submodule update --init --recursive --depth 1 --force
  # install-prereqs installs its own Python venv (~/venv-ardupilot) and pymavlink/MAVProxy.
  USER="${USER:-$(id -un)}" Tools/environment_install/install-prereqs-ubuntu.sh -y
  # shellcheck disable=SC1090
  source "$HOME/.profile" || true
  ./waf configure --board sitl
  ./waf copter
fi

# -- ardupilot_gazebo plugin ------------------------------------------------------------
if [ ! -f "$SIM_HOME/ardupilot_gazebo/build/libArduPilotPlugin.so" ]; then
  say "ardupilot_gazebo plugin (Gazebo Harmonic)"
  [ -d "$SIM_HOME/ardupilot_gazebo/.git" ] || git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git "$SIM_HOME/ardupilot_gazebo"
  mkdir -p "$SIM_HOME/ardupilot_gazebo/build"
  cd "$SIM_HOME/ardupilot_gazebo/build"
  GZ_VERSION=harmonic cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
  make -j"$(nproc)"
fi

# -- onboard-autonomy into its own overlay (same packages as the Jetson) -------------------
say "build onboard-autonomy into $OVERLAY_WS"
set +u
source /opt/ros/jazzy/setup.bash
set -u
mkdir -p "$OVERLAY_WS"
cd "$OVERLAY_WS"
colcon build --symlink-install \
  --base-paths "$NIDAR_DIR/onboard-autonomy" \
  --packages-select nidar_airmouse nidar_autonomy
chmod +x "$NIDAR_DIR"/scripts/sim/*.sh "$NIDAR_DIR"/missions/*/mission.py

echo
echo "Done. Start the sim with:  $NIDAR_DIR/scripts/sim/start_sim.sh"
echo "then the GCS on the laptop (see README section 7.5) and START 'Main Mission (Sim)'."
