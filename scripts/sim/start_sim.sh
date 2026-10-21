#!/usr/bin/env bash
# Main Mission (Sim): the whole drone side in one terminal, on a laptop.
#
#   Gazebo Harmonic   Rish's NIDAR maze + iris with a 360 deg 2D LiDAR (sim/)
#   ArduCopter SITL   4.6.3, the drone's firmware, flow + rangefinder, no GPS
#   MAVROS            tcp://127.0.0.1:5760 (SITL's first serial port)
#   ros_gz_bridge     /scan (frame "laser") and /clock
#   Cartographer      the drone's own 2D config (scripts/jetson/cartographer)
#   Jetson nodes      command_node, heartbeat_node, radio_command_node
#   missions/main_mission/mission.py --sim
#   socat             the radio pair: /tmp/nidar_radio  <->  TCP port 5800
#
# Then on the laptop start the GCS with the "radio" on TCP and START
# "Main Mission (Sim)" -- same panel, same radio protocol as the real drone:
#   Windows:  .\scripts\gcs\start_gcs.ps1 -RadioPort socket://127.0.0.1:5800
#   Linux:    ./scripts/gcs/start_gcs.sh --port socket://127.0.0.1:5800
#
# Usage:
#   start_sim.sh           headless Gazebo (works in WSL / over SSH)
#   start_sim.sh --gui     also open the Gazebo window (watch the drone)
# Ctrl+C stops everything. Logs: ~/NidarFiles/logs/sim_<time>/ (or the repo's logs/)
set -uo pipefail
[ "$(id -u)" -eq 0 ] && { echo "ERROR: run as your normal user, not with sudo." >&2; exit 1; }

NIDAR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SIM_HOME="${NIDAR_SIM_HOME:-$HOME/nidar_sim}"
OVERLAY_WS="${NIDAR_SIM_WS:-$HOME/nidar_sim_ws}"
ARDUPILOT="$SIM_HOME/ardupilot"
AP_GAZEBO="$SIM_HOME/ardupilot_gazebo"
GEN="$SIM_HOME/generated"
RADIO_PTY="/tmp/nidar_radio"
RADIO_TCP_PORT="${NIDAR_SIM_RADIO_PORT:-5800}"
# EKF origin / SITL home: the same point mission.py gives the drone (Vellore)
HOME_LOC="12.9165,79.1325,216,90"

GUI=false
for arg in "$@"; do
  case "$arg" in
    --gui) GUI=true ;;
    *) echo "unknown option: $arg (use --gui)"; exit 1 ;;
  esac
done

LOG_DIR="$NIDAR_DIR/logs/sim_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
PIDS=()
say() { echo "[$(date +%H:%M:%S)] $*"; }
fail() { say "STOP: $*"; exit 1; }
cleanup() {
  say "stopping ${#PIDS[@]} processes"
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
  sleep 2
  for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null; done
  pkill -f "gz sim.*nidar_main_mission" 2>/dev/null
  rm -f "$RADIO_PTY"
}
trap cleanup EXIT
run() {  # run <name> <command...>  -- background, logged
  local name=$1; shift
  "$@" >"$LOG_DIR/$name.log" 2>&1 &
  PIDS+=($!)
  say "started $name (pid $!, log $LOG_DIR/$name.log)"
}

# -- environment ----------------------------------------------------------------
[ -x "$ARDUPILOT/build/sitl/bin/arducopter" ] || fail "ArduCopter SITL not built -- run scripts/sim/setup_sim.sh"
[ -f "$AP_GAZEBO/build/libArduPilotPlugin.so" ] || fail "ardupilot_gazebo not built -- run scripts/sim/setup_sim.sh"
set +u
source /opt/ros/jazzy/setup.bash || fail "ROS 2 Jazzy not found -- run scripts/sim/setup_sim.sh"
[ -f "$OVERLAY_WS/install/setup.bash" ] || fail "$OVERLAY_WS not built -- run scripts/sim/setup_sim.sh"
source "$OVERLAY_WS/install/setup.bash"
set -u
python3 -c "import nidar_autonomy.radio_command_node" 2>/dev/null || fail "nidar_autonomy not importable -- rerun scripts/sim/setup_sim.sh"
# Sourcing ROS puts its vendored gz tools first (PATH and GZ_CONFIG_PATH): they
# have topic/service but no "gz sim" (gps_denied README: "the gz PATH shadow").
# Gazebo Harmonic's own gz and config must win.
export PATH="/usr/bin:$PATH"
export GZ_CONFIG_PATH="/usr/share/gz${GZ_CONFIG_PATH:+:$GZ_CONFIG_PATH}"
gz sim --help >/dev/null 2>&1 || fail "'gz sim' not available -- is gz-harmonic installed (setup_sim.sh)?"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$AP_GAZEBO/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$GEN/models:$AP_GAZEBO/models:$AP_GAZEBO/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export NIDAR_SIM=1  # mission.py: only the Sim mission may start here

OURS='gz sim.*nidar_main_mission|arducopter --model JSON|lib/nidar_autonomy/(radio_command_node|command_node|heartbeat_node)|missions/(hover|motor_test|main_mission)/mission\.py|cartographer_(node|occupancy_grid_node)|socat .*nidar_radio'
if pgrep -f "$OURS" >/dev/null; then
  say "STOP: parts of an earlier sim (or the drone stack) are still running:"
  pgrep -af "$OURS" | sed 's/^/    /'
  fail "stop them first:  pkill -INT -f '$OURS'"
fi

# -- Gazebo world (rebuilt each start from sim/worlds/nidar_sim.sdf) ---------------------
python3 "$NIDAR_DIR/sim/tools/make_sim_world.py" --ardupilot-gazebo "$AP_GAZEBO" --out "$GEN" >"$LOG_DIR/make_world.log" 2>&1 \
  || fail "could not build the sim world (see $LOG_DIR/make_world.log)"
WORLD="$GEN/worlds/nidar_main_mission.sdf"
if $GUI; then
  run gazebo gz sim -r "$WORLD"
else
  run gazebo gz sim -r -s --headless-rendering "$WORLD"
fi
say "waiting for the Gazebo world (60 s)"
for _ in $(seq 1 60); do
  gz topic -l 2>/dev/null | grep -q "^/scan$" && break
  sleep 1
done
gz topic -l 2>/dev/null | grep -q "^/scan$" || fail "Gazebo world/LiDAR did not come up (see $LOG_DIR/gazebo.log)"
say "Gazebo up: maze + drone at the entry, LiDAR on /scan"

# -- ArduCopter SITL (JSON link to the ArduPilotPlugin in the drone model) -------------------
mkdir -p "$SIM_HOME/sitl"
DEFAULTS="$ARDUPILOT/Tools/autotest/default_params/copter.parm,$ARDUPILOT/Tools/autotest/default_params/gazebo-iris.parm,$NIDAR_DIR/sim/params/nidar_sitl.parm"
( cd "$SIM_HOME/sitl" && exec "$ARDUPILOT/build/sitl/bin/arducopter" --model JSON --speedup 1 -I0 \
    --home "$HOME_LOC" --defaults "$DEFAULTS" ) >"$LOG_DIR/sitl.log" 2>&1 &
PIDS+=($!)
say "started sitl (pid $!, log $LOG_DIR/sitl.log)"

# -- MAVROS ----------------------------------------------------------------------
run mavros ros2 launch mavros apm.launch "fcu_url:=tcp://127.0.0.1:5760"
say "waiting for /mavros/state connected=true (60 s)"
connected=false
for _ in $(seq 1 30); do
  if timeout 3 ros2 topic echo --once /mavros/state 2>/dev/null | grep -q "connected: true"; then
    connected=true; break
  fi
  sleep 1
done
$connected || fail "MAVROS not connected to SITL (see $LOG_DIR/sitl.log and mavros.log)"
say "MAVROS connected to ArduCopter SITL"

# -- LiDAR + clock into ROS, Cartographer (the drone's config, sim time) ---------------------
run gz_bridge ros2 run ros_gz_bridge parameter_bridge \
  "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
CARTO_SHARE="$(ros2 pkg prefix cartographer_ros)/share/cartographer_ros/configuration_files"
CARTO_DIR="$LOG_DIR/cartographer"
mkdir -p "$CARTO_DIR"
cp "$CARTO_SHARE/revo_lds.lua" "$NIDAR_DIR/scripts/jetson/cartographer/nidar_2d.lua" "$CARTO_DIR/"
sed -i "s/^TRAJECTORY_BUILDER_2D.min_range = .*/TRAJECTORY_BUILDER_2D.min_range = 0.3/" "$CARTO_DIR/nidar_2d.lua"
run cartographer ros2 run cartographer_ros cartographer_node \
  -configuration_directory "$CARTO_DIR" -configuration_basename nidar_2d.lua --ros-args -p use_sim_time:=true
run cartographer_grid ros2 run cartographer_ros cartographer_occupancy_grid_node \
  -resolution 0.1 -publish_period_sec 1.0 --ros-args -p use_sim_time:=true

# -- the "radio": a pseudo serial port for the Jetson node, TCP for the GCS ----------------
( while true; do
    socat "pty,link=$RADIO_PTY,raw,echo=0" "tcp-listen:$RADIO_TCP_PORT,reuseaddr" || sleep 1
  done ) >"$LOG_DIR/socat.log" 2>&1 &
PIDS+=($!)
say "radio stand-in: $RADIO_PTY <-> tcp port $RADIO_TCP_PORT"

# -- Jetson nodes + the mission -----------------------------------------------------------
run command_node ros2 run nidar_autonomy command_node
run heartbeat_node ros2 run nidar_autonomy heartbeat_node
run radio_command_node ros2 run nidar_autonomy radio_command_node --ros-args \
  -p "serial_port:=$RADIO_PTY" -p "fallback_serial_port:=$RADIO_PTY" -p "baud:=115200" \
  -p "dry_run:=false" -p "telemetry_rate_hz:=2.0" -p "allow_param_write:=false" \
  -p "lidar_rate_hz:=1.0" -p "map_rate_hz:=2.0" -p "map_cell_m:=0.25"
run main_mission_sim python3 "$NIDAR_DIR/missions/main_mission/mission.py" --sim --ros-args -p use_sim_time:=true

# ArduCopter arms only once the EKF origin (set by mission.py) has become a
# home position and the gyros have settled -- in a sim slower than real time
# that takes a while. Don't announce READY before it.
say "waiting for the FCU home position (EKF origin from the mission, up to 3 min)"
home=false
for _ in $(seq 1 60); do
  if timeout 3 ros2 topic echo --once /mavros/home_position/home >/dev/null 2>&1; then
    home=true; break
  fi
  sleep 2
done
$home || say "WARNING: no home position yet -- START may be refused ('waiting for home'); see $LOG_DIR/main_mission_sim.log"

say "-------------------------------------------------------------------"
say "SIM READY. On the laptop start the GCS with the radio on TCP:"
say "  Windows: .\\scripts\\gcs\\start_gcs.ps1 -RadioPort socket://127.0.0.1:$RADIO_TCP_PORT"
say "  Linux:   ./scripts/gcs/start_gcs.sh --port socket://127.0.0.1:$RADIO_TCP_PORT"
say "then choose 'Main Mission (Sim)' and press START. Ctrl+C here stops the sim."
say "-------------------------------------------------------------------"
tail -n +1 -F "$LOG_DIR/main_mission_sim.log" "$LOG_DIR/radio_command_node.log" &
PIDS+=($!)
wait
