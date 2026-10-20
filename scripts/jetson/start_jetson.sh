#!/usr/bin/env bash
# Starts the complete Jetson side of the NIDAR hover in one terminal:
#
#   MAVROS (Ethernet to the Pixhawk)
#   command_node + heartbeat_node    (existing onboard-autonomy nodes)
#   radio_command_node               (MicroLR900 radio, USB serial: START/ABORT
#                                     in, telemetry out -- the GCS's only link)
#   missions/hover/mission.py        (the hover mission -- REAL FLIGHT)
#   missions/motor_test/mission.py   (motor test -- spins motors, PROPS OFF)
#
# There is no Wi-Fi link to the GCS and no rosbridge: everything the GCS
# shows comes over the radio.
#
# Does NOT start mission_state_node: it would arm on the same START by
# itself. Refuses to run if it is already running.
#
# Usage:
#   start_jetson.sh              EVERYTHING (a radio START will fly the hover):
#                                the RPLIDAR A2 scan to the GCS LiDAR panel and
#                                Cartographer 2D SLAM -- live map (and the
#                                LiDAR's position on it) to the GCS Map panel.
#                                With the Pixhawk connected, its IMU is used in
#                                the map too (if it streams fast enough;
#                                NIDAR_MAP_IMU=off for LiDAR only). A missing
#                                LiDAR or Cartographer is a warning, not a stop.
#   start_jetson.sh --no-map     no SLAM map (LiDAR panel still on)
#   start_jetson.sh --no-lidar   no LiDAR at all (so no map either)
#   start_jetson.sh --no-fcu     no Pixhawk connected: skip the Pixhawk/MAVROS
#                                checks; radio link + LiDAR/map only, START
#                                refused. Hand-held mapping.
#   start_jetson.sh --setup      also let the GCS Setup page write Pixhawk
#                                parameters (bench only; refused while armed)
#   start_jetson.sh --dry-run    radio node ACKs and logs but publishes nothing;
#                                nothing can arm. Use for the first radio test.
# Options combine, e.g. --dry-run --no-map. (--lidar / --map are still
# accepted; they are the default now.)
#
# Ctrl+C stops everything this script started. Logs: ~/NidarFiles/logs/<time>/
#
# Values below are the documented setup (custom-gcs Daily Startup SOP);
# override with environment variables if the hardware differs.
set -uo pipefail
[ "$(id -u)" -eq 0 ] && { echo "ERROR: run as your normal user, not with sudo." >&2; exit 1; }

NIDAR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PIXHAWK_IP="${NIDAR_PIXHAWK_IP:-192.168.144.14}"
PIXHAWK_PORT="${NIDAR_PIXHAWK_PORT:-14550}"
JETSON_ETH_IP="${NIDAR_JETSON_ETH_IP:-192.168.144.1}"
ETH_IF="${NIDAR_ETH_IF:-eno1}"
RADIO_PORT="${NIDAR_RADIO_PORT:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
RADIO_FALLBACK_PORT="${NIDAR_RADIO_FALLBACK_PORT:-/dev/ttyUSB0}"
RADIO_BAUD="${NIDAR_RADIO_BAUD:-115200}"
# Position/attitude rate over the radio (FCU state, battery: 1 Hz). Keep it
# low: START/ABORT ACKs share the radio's air time.
# TODO(hardware): raise only after measuring the radio's real throughput.
TELEMETRY_RATE_HZ="${NIDAR_TELEMETRY_RATE_HZ:-2.0}"
# LiDAR: scans sent over the radio per second (~180 B each), and the
# direction of the LiDAR's 0 deg mark relative to the drone's nose (clockwise).
LIDAR_RATE_HZ="${NIDAR_LIDAR_RATE_HZ:-1.0}"
LIDAR_YAW_DEG="${NIDAR_LIDAR_YAW_DEG:-0}"
# Map: packets per second (~145 B each) and the cell size sent to the GCS.
MAP_RATE_HZ="${NIDAR_MAP_RATE_HZ:-2.0}"
MAP_CELL_M="${NIDAR_MAP_CELL_M:-0.25}"
# Ignore LiDAR returns closer than this (m): the person holding it, or the
# drone's own frame/props. Applies to the LiDAR panel and the map.
LIDAR_MIN_RANGE_M="${NIDAR_LIDAR_MIN_RANGE_M:-0.4}"
# Map with the Pixhawk's IMU: auto = when the Pixhawk is connected and
# /mavros/imu/data arrives at >= MAP_IMU_MIN_HZ; off = LiDAR only (e.g. the
# Pixhawk sits on the bench while the LiDAR is carried around: its IMU would
# say "not moving" and wreck the map). The LiDAR's position relative to the
# Pixhawk (m, x forward, y left, z up) -- only used with the IMU.
MAP_IMU="${NIDAR_MAP_IMU:-auto}"
MAP_IMU_MIN_HZ="${NIDAR_MAP_IMU_MIN_HZ:-30}"
LIDAR_X_M="${NIDAR_LIDAR_X_M:-0}"
LIDAR_Y_M="${NIDAR_LIDAR_Y_M:-0}"
LIDAR_Z_M="${NIDAR_LIDAR_Z_M:-0}"

DRY_RUN=false
SETUP=false
LIDAR=true
NO_FCU=false
MAP=true
for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=true ;;
    --setup) SETUP=true ;;
    --no-fcu) NO_FCU=true ;;
    --no-map) MAP=false ;;
    --no-lidar) LIDAR=false ;;
    --lidar|--map) ;;  # on by default now
    *) echo "unknown option: $arg (use --dry-run, --setup, --no-fcu, --no-map, --no-lidar)"; exit 1 ;;
  esac
done
$LIDAR || MAP=false  # the map is built from the LiDAR

LOG_DIR="$NIDAR_DIR/logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
PIDS=()

say() { echo "[$(date +%H:%M:%S)] $*"; }
fail() { say "STOP: $*"; exit 1; }

# ros2 reads "-p name:=270" as an INTEGER, and a node that declares the
# parameter as a double then refuses it (InvalidParameterTypeException) and
# exits. Every float-typed parameter passed below goes through ros_double,
# which turns a whole number into a decimal ("270" -> "270.0") and rejects
# anything that isn't a plain number.
ros_double() {
  local v="$1"
  [[ "$v" =~ ^[+-]?[0-9]+(\.[0-9]+)?$ ]] || return 1
  [[ "$v" == *.* ]] && echo "$v" || echo "$v.0"
}
TELEMETRY_RATE_HZ="$(ros_double "$TELEMETRY_RATE_HZ")" || fail "NIDAR_TELEMETRY_RATE_HZ must be a number (e.g. 2 or 2.0)"
LIDAR_RATE_HZ="$(ros_double "$LIDAR_RATE_HZ")" || fail "NIDAR_LIDAR_RATE_HZ must be a number (e.g. 1 or 0.5)"
LIDAR_YAW_DEG="$(ros_double "$LIDAR_YAW_DEG")" || fail "NIDAR_LIDAR_YAW_DEG must be a number of degrees (e.g. 0, 90, 180, 270)"
MAP_RATE_HZ="$(ros_double "$MAP_RATE_HZ")" || fail "NIDAR_MAP_RATE_HZ must be a number (e.g. 2 or 1.0)"
MAP_CELL_M="$(ros_double "$MAP_CELL_M")" || fail "NIDAR_MAP_CELL_M must be a number of metres (e.g. 0.25)"
LIDAR_MIN_RANGE_M="$(ros_double "$LIDAR_MIN_RANGE_M")" || fail "NIDAR_LIDAR_MIN_RANGE_M must be a number of metres (e.g. 0.4)"
case "$MAP_IMU" in auto|off) ;; *) fail "NIDAR_MAP_IMU must be auto or off" ;; esac
MAP_IMU_MIN_HZ="$(ros_double "$MAP_IMU_MIN_HZ")" || fail "NIDAR_MAP_IMU_MIN_HZ must be a number (e.g. 30)"
LIDAR_X_M="$(ros_double "$LIDAR_X_M")" || fail "NIDAR_LIDAR_X_M must be a number of metres (e.g. 0.05)"
LIDAR_Y_M="$(ros_double "$LIDAR_Y_M")" || fail "NIDAR_LIDAR_Y_M must be a number of metres (e.g. 0)"
LIDAR_Z_M="$(ros_double "$LIDAR_Z_M")" || fail "NIDAR_LIDAR_Z_M must be a number of metres (e.g. 0.1)"

cleanup() {
  say "stopping ${#PIDS[@]} processes"
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
  sleep 2
  for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null; done
}
trap cleanup EXIT

run() {  # run <name> <command...>  -- background, logged
  local name=$1; shift
  "$@" >"$LOG_DIR/$name.log" 2>&1 &
  PIDS+=($!)
  say "started $name (pid $!, log $LOG_DIR/$name.log)"
}

# -- environment ----------------------------------------------------------------
set +u  # ROS setup files read unset variables
source /opt/ros/humble/setup.bash || fail "ROS 2 Humble not found"
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
[ -f "$HOME/nidar_ws/install/setup.bash" ] || fail "~/nidar_ws not built -- run scripts/jetson/setup_jetson.sh"
source "$HOME/nidar_ws/install/setup.bash"
python3 -c "import nidar_autonomy.radio_command_node" 2>/dev/null || fail "nidar_autonomy (with radio_command_node) not importable -- rebuild with setup_jetson.sh"
set -u

# -- USB serial ports: radio vs LiDAR (before anything opens the radio) ---------
# The RPLIDAR's USB adapter and the radio are both CP2102 chips. On our units
# their /dev/serial/by-id names differ (the LiDAR's has a long unique serial,
# the radio's ends in "Controller_0001"), but CP2102 adapters MAY share a
# name, so the port choice does not rely on names: whenever more than one USB
# serial port is present (or the LiDAR is wanted), ask each port which one
# answers like an RPLIDAR (GET_INFO) and give the radio another port -- so a
# plugged-in LiDAR can never take the radio's place, even with --no-lidar.
LIDAR_PORT=""
USB_SERIAL_COUNT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | wc -l)
if $LIDAR || [ "$USB_SERIAL_COUNT" -gt 1 ]; then
  say "checking $USB_SERIAL_COUNT USB serial port(s) for the RPLIDAR..."
  eval "$(python3 -m nidar_autonomy.rplidar_probe)"
  if [ -z "$LIDAR_PORT" ]; then
    $LIDAR && say "WARNING: no RPLIDAR answered (ports: ${OTHER_PORTS:-none}) -- continuing WITHOUT LiDAR"
    LIDAR=false
  else
    say "RPLIDAR on $LIDAR_PORT @ $LIDAR_BAUD ($LIDAR_INFO)$($LIDAR || echo ' -- not used (--no-lidar)')"
    if [ -n "${NIDAR_RADIO_PORT:-}" ]; then
      [ "$(readlink -f "$NIDAR_RADIO_PORT")" = "$LIDAR_PORT" ] && fail "NIDAR_RADIO_PORT=$NIDAR_RADIO_PORT is the LiDAR, not the radio"
    else
      read -r FIRST_OTHER _ <<<"${OTHER_PORTS:-}"
      if [ -n "${FIRST_OTHER:-}" ]; then
        RADIO_PORT="$FIRST_OTHER"
        RADIO_FALLBACK_PORT="$FIRST_OTHER"
        say "radio -> $RADIO_PORT (the USB serial port that is not the LiDAR)"
      else
        say "WARNING: only the LiDAR is plugged in -- no USB serial port left for the radio"
      fi
    fi
  fi
fi

# -- an earlier stack still running? -----------------------------------------------
# Leftovers hold the radio/LiDAR ports (the probe then reports "no RPLIDAR
# answered") and a second radio_command_node would fight over the radio.
# MAVROS is not in this list: it is reused on purpose.
OURS='lib/nidar_autonomy/(radio_command_node|lidar_node|command_node|heartbeat_node)|missions/(hover|motor_test)/mission\.py|cartographer_(node|occupancy_grid_node)|static_transform_publisher.*laser'
if pgrep -f "$OURS" >/dev/null; then
  say "STOP: parts of an earlier stack are still running:"
  pgrep -af "$OURS" | sed 's/^/    /'
  fail "stop them first:  pkill -INT -f '$OURS'   (then run this again)"
fi

# -- safety: one authoritative execution path ----------------------------------
if pgrep -f "nidar_autonomy/mission_state_node|lib/nidar_autonomy/mission_state_node" >/dev/null; then
  fail "mission_state_node is running -- it would arm on START by itself. Stop it first."
fi

if $NO_FCU; then
  say "NO-FCU MODE: Pixhawk and MAVROS skipped -- radio, telemetry heartbeat and LiDAR only; START is rejected (FCU_NOT_CONNECTED)"
else
# -- Pixhawk link ---------------------------------------------------------------
if ip -br link show "$ETH_IF" 2>/dev/null | grep -qw DOWN; then
  say "$ETH_IF is DOWN -- bringing it up"
  sudo ip link set "$ETH_IF" up || fail "could not bring $ETH_IF up"
  sleep 2
fi
if ! ip -4 addr show "$ETH_IF" | grep -q "$JETSON_ETH_IP"; then
  say "$ETH_IF lacks $JETSON_ETH_IP -- restoring (runtime only, see SOP Phase 3)"
  sudo ip addr add "$JETSON_ETH_IP/24" dev "$ETH_IF" || fail "could not set $JETSON_ETH_IP on $ETH_IF"
fi
ping -c 2 -W 1 "$PIXHAWK_IP" >/dev/null 2>&1 || fail "Pixhawk $PIXHAWK_IP does not answer on $ETH_IF (no Pixhawk connected? use --no-fcu for radio/LiDAR only)"
say "Pixhawk reachable at $PIXHAWK_IP"

# -- MAVROS ---------------------------------------------------------------------
if pgrep -f "mavros_node" >/dev/null; then
  say "MAVROS already running -- reusing it"
else
  run mavros ros2 launch mavros apm.launch "fcu_url:=udp://@$PIXHAWK_IP:$PIXHAWK_PORT"
fi
say "waiting for /mavros/state connected=true (30 s)"
connected=false
for _ in $(seq 1 15); do
  if timeout 3 ros2 topic echo --once /mavros/state 2>/dev/null | grep -q "connected: true"; then
    connected=true; break
  fi
  sleep 1
done
$connected || fail "MAVROS not connected to the Pixhawk (see $LOG_DIR/mavros.log)"
say "MAVROS connected"
fi

[ -e "$RADIO_PORT" ] || [ -e "$RADIO_FALLBACK_PORT" ] || say "WARNING: radio not found ($RADIO_PORT / $RADIO_FALLBACK_PORT) -- radio node will keep retrying"

# -- onboard-autonomy nodes -------------------------------------------------------
run command_node ros2 run nidar_autonomy command_node
run heartbeat_node ros2 run nidar_autonomy heartbeat_node
run radio_command_node ros2 run nidar_autonomy radio_command_node --ros-args \
  -p "serial_port:=$RADIO_PORT" -p "fallback_serial_port:=$RADIO_FALLBACK_PORT" \
  -p "baud:=$RADIO_BAUD" -p "dry_run:=$DRY_RUN" -p "telemetry_rate_hz:=$TELEMETRY_RATE_HZ"   -p "allow_param_write:=$SETUP" -p "lidar_rate_hz:=$LIDAR_RATE_HZ" \
  -p "map_rate_hz:=$MAP_RATE_HZ" -p "map_cell_m:=$MAP_CELL_M"
run hover_mission python3 "$NIDAR_DIR/missions/hover/mission.py"
run motor_test_mission python3 "$NIDAR_DIR/missions/motor_test/mission.py"
if $LIDAR; then
  run lidar_node ros2 run nidar_autonomy lidar_node --ros-args \
    -p "serial_port:=$LIDAR_PORT" -p "baud:=$LIDAR_BAUD" -p "yaw_offset_deg:=$LIDAR_YAW_DEG" \
    -p "range_min_m:=$LIDAR_MIN_RANGE_M"
fi
if $MAP && ! $LIDAR; then
  say "WARNING: no LiDAR -- no map"
  MAP=false
elif $MAP; then
  # Cartographer 2D SLAM from /scan (LiDAR only). Our config includes
  # Cartographer's own hand-held example, copied from the installed package.
  CARTO_SHARE="$(ros2 pkg prefix cartographer_ros 2>/dev/null)/share/cartographer_ros/configuration_files"
  if [ ! -f "$CARTO_SHARE/revo_lds.lua" ]; then
    say "WARNING: Cartographer not installed -- no map (run setup_jetson.sh with internet)"
    MAP=false
  else
    CARTO_DIR="$LOG_DIR/cartographer"
    mkdir -p "$CARTO_DIR"
    cp "$CARTO_SHARE/revo_lds.lua" "$NIDAR_DIR/scripts/jetson/cartographer/nidar_2d.lua" \
      "$NIDAR_DIR/scripts/jetson/cartographer/nidar_2d_imu.lua" "$CARTO_DIR/"
    sed -i "s/^TRAJECTORY_BUILDER_2D.min_range = .*/TRAJECTORY_BUILDER_2D.min_range = $LIDAR_MIN_RANGE_M/" "$CARTO_DIR/nidar_2d.lua"
    CARTO_CONFIG=nidar_2d.lua
    CARTO_ARGS=()
    SLAM_KIND="LiDAR only"
    if [ "$MAP_IMU" = off ]; then
      say "MAP: NIDAR_MAP_IMU=off -- LiDAR-only SLAM"
    elif $NO_FCU; then
      say "MAP: no Pixhawk -- LiDAR-only SLAM"
    else
      # Stream rates reset whenever the Pixhawk/MAVROS restarts: ask for the
      # IMU at 50 Hz -- ATTITUDE (30) drives /mavros/imu/data, RAW_IMU (27)
      # gives it its acceleration -- then measure what actually arrives.
      for msg_id in 30 27; do
        timeout 10 ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval \
          "{message_id: $msg_id, message_rate: 50.0}" >/dev/null 2>&1 \
          || say "WARNING: could not ask the Pixhawk for MAVLink message $msg_id at 50 Hz"
      done
      say "MAP: measuring the Pixhawk IMU rate (8 s)"
      IMU_HZ="$(timeout 8 ros2 topic hz /mavros/imu/data 2>/dev/null | awk '/average rate/ {r = $3} END {print r + 0}')"
      if awk -v r="$IMU_HZ" -v m="$MAP_IMU_MIN_HZ" 'BEGIN {exit !(r >= m)}'; then
        CARTO_CONFIG=nidar_2d_imu.lua
        CARTO_ARGS=(--ros-args -r imu:=/mavros/imu/data)
        SLAM_KIND="LiDAR + Pixhawk IMU"
        run lidar_tf ros2 run tf2_ros static_transform_publisher \
          --x "$LIDAR_X_M" --y "$LIDAR_Y_M" --z "$LIDAR_Z_M" --frame-id base_link --child-frame-id laser
        say "MAP: using the Pixhawk IMU (/mavros/imu/data at $IMU_HZ Hz) -- the Pixhawk must move WITH the LiDAR"
      else
        say "WARNING: Pixhawk IMU at $IMU_HZ Hz (need $MAP_IMU_MIN_HZ) -- LiDAR-only SLAM"
      fi
    fi
    run cartographer ros2 run cartographer_ros cartographer_node \
      -configuration_directory "$CARTO_DIR" -configuration_basename "$CARTO_CONFIG" "${CARTO_ARGS[@]}"
    run cartographer_grid ros2 run cartographer_ros cartographer_occupancy_grid_node \
      -resolution 0.1 -publish_period_sec 1.0
    say "MAP: Cartographer SLAM ($SLAM_KIND) running -> /map -> GCS Map panel (${MAP_CELL_M} m cells)"
  fi
fi

say "-------------------------------------------------------------------"
if $SETUP; then
  say "SETUP: the GCS Setup page may WRITE Pixhawk parameters (refused while armed)."
fi
if $DRY_RUN; then
  say "DRY RUN: radio commands are ACKed and logged, nothing is published, nothing can arm."
else
  say "LIVE: a radio START (Hover) from the GCS WILL arm and fly the vehicle;"
  say "      a radio START (Motor Test) WILL spin the motors -- PROPS OFF."
fi
say "Ctrl+C stops everything. Following radio + mission logs:"
say "-------------------------------------------------------------------"
LOGS=("$LOG_DIR/radio_command_node.log" "$LOG_DIR/hover_mission.log" "$LOG_DIR/motor_test_mission.log")
$LIDAR && LOGS+=("$LOG_DIR/lidar_node.log")
$MAP && LOGS+=("$LOG_DIR/cartographer.log")
tail -n +1 -F "${LOGS[@]}" &
PIDS+=($!)
wait
