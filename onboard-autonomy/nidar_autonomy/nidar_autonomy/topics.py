"""Shared topic name / value constants, kept in one place so the three
nodes (and any future ones) can't drift apart from each other or from
custom-gcs/docs/DATA_MODELS.md, which is the actual source of truth."""

COMMAND_TOPIC = "/gcs/command"
MISSION_STATE_TOPIC = "/mission/state"
HEARTBEAT_TOPIC = "/gcs/heartbeat"

# Internal-only topic, not part of the custom-gcs interface contract.
# command_node publishes here ONLY after validating a /gcs/command message
# is exactly "start" or "abort" -- mission_state_node listens to this, never
# to the raw /gcs/command, so an invalid/malformed message can never reach
# the state machine even if command_node's validation is ever bypassed
# elsewhere. See Hard Safety Rule 5 in this repo's CLAUDE.md.
VALIDATED_COMMAND_TOPIC = "/nidar_autonomy/validated_command"

VALID_COMMANDS = ("start", "abort")

# Per custom-gcs/docs/DATA_MODELS.md section 1.
MISSION_STATES = ("idle", "entering", "searching", "exiting", "complete", "aborted")

# -- Mapping/exploration/telemetry topics -----------------------------------
#
# Added by the NIDAR Autonomy Migration (folding gps_denied/raj-dev's
# mapping/exploration/telemetry concepts into this repo -- see
# CHECKPOINT/CURRENT_STATE.md). Names match gps_denied's own
# docs/gcs_telemetry_contract.md, adopted here as the new canonical
# contract per that migration's discovery pass -- see
# CHECKPOINT/CURRENT_STATE.md / custom-gcs/docs/DATA_MODELS.md for the
# joint custom-gcs interface this supersedes (`/slam/map`, never
# implemented, is replaced by `/map` below).

# Already published by mavros itself once it's running -- read-only
# subscriptions, never published to from this repo except where noted.
FCU_STATE_TOPIC = "/mavros/state"
BATTERY_TOPIC = "/mavros/battery"
# Relayed to the GCS over the radio by radio_command_node (radio_telemetry.py).
LOCAL_POSITION_TOPIC = "/mavros/local_position/pose"
LOCAL_VELOCITY_TOPIC = "/mavros/local_position/velocity_local"
IMU_TOPIC = "/mavros/imu/data"
FCU_STATUSTEXT_TOPIC = "/mavros/statustext/recv"
SCAN_TOPIC = "/scan"

# Localization: a future SLAM pipeline's pose estimate, fed back to the FCU's
# EKF as an external vision position estimate (AUTONOMY_ROADMAP.md Phase 4).
# Not yet published by anything in this repo -- see vision_pose_bridge_node.py
# migration status in the migration report.
VISION_POSE_TOPIC = "/mavros/vision_pose/pose"
SLAM_OK_TOPIC = "/slam_ok"

# Mapping (nav_msgs/OccupancyGrid, native topics -- not duplicated into the
# normalized telemetry contract below; see telemetry_contract.py's module
# docstring for why).
MAP_TOPIC = "/map"
COVERAGE_GRID_TOPIC = "/coverage_grid"
COVERAGE_PERCENT_TOPIC = "/coverage/percent"

# Exploration/planning (native topics).
FRONTIERS_TOPIC = "/frontiers"
PLANNED_PATH_TOPIC = "/planned_path"
EXPLORER_STATUS_TOPIC = "/explorer/status"

# Safety (this repo's own local-frame substitute for ArduCopter's
# GPS-based native geofence -- see geofence_monitor_node.py).
GEOFENCE_BREACH_TOPIC = "/mission/geofence_breach"

# The normalized, single-JSON-blob telemetry contract for the GCS -- see
# telemetry_contract.py and telemetry_bridge_node.py.
TELEMETRY_STATE_TOPIC = "/telemetry/state"

# -- Simulation-only topics ---------------------------------------------------
#
# Added for the GCS "RUN SIMULATION" path (see
# CHECKPOINT/CURRENT_STATE.md and mission_simulator.py /
# simulation_node.py). Every name below lives under the `/simulation/`
# namespace specifically so it can NEVER collide with the real topics
# above -- simulation_node.py never subscribes to or publishes any
# non-`/simulation/`-prefixed topic, and never imports mavros_msgs or
# flight_command.py. `/simulation/command` is a completely separate
# channel from the real `/gcs/command`; sending "run"/"reset" here has no
# effect on the real mission_state_node.py or the real vehicle.
SIMULATION_COMMAND_TOPIC = "/simulation/command"
SIMULATION_MISSION_STATE_TOPIC = "/simulation/mission/state"
SIMULATION_STATUS_TOPIC = "/simulation/status"
SIMULATION_MAP_TOPIC = "/simulation/map"
SIMULATION_COVERAGE_GRID_TOPIC = "/simulation/coverage_grid"
SIMULATION_PLANNED_PATH_TOPIC = "/simulation/planned_path"
SIMULATION_TELEMETRY_STATE_TOPIC = "/simulation/telemetry/state"

VALID_SIMULATION_COMMANDS = ("run", "reset")

# -- Radio command path + hover mission ---------------------------------------
#
# radio_command_node.py receives START/ABORT over the MicroLR900 radio and
# publishes them on COMMAND_TOPIC above (through command_node's normal
# validation). Before a START it publishes MISSION_SELECT_TOPIC
# ({"mission_id": "hover", ...}) -- routing metadata, not a command: the
# hover mission only acts on a "start" that was immediately preceded by a
# selection naming it.
MISSION_SELECT_TOPIC = "/gcs/mission_select"

# The hover mission's status (JSON std_msgs/String, ~2 Hz). Same topic and
# field names the GCS already displays for a hover flight test
# (custom-gcs FlightTestStatusResponse), with execution_mode "real".
# radio_command_node also uses it as the mission's liveness signal.
HOVER_STATUS_TOPIC = "/flight_test/status"

# The motor test mission's status (JSON std_msgs/String, ~2 Hz), same shape
# as HOVER_STATUS_TOPIC with scenario "motor_test". Also its liveness signal.
MOTOR_TEST_STATUS_TOPIC = "/motor_test/status"

# The Main Mission's status (missions/main_mission, JSON std_msgs/String,
# ~2 Hz): the real drone, and the same mission flying ArduPilot SITL in
# Gazebo (scripts/sim/start_sim.sh). Same shape as HOVER_STATUS_TOPIC plus
# exploration fields; also each mission's liveness signal for
# radio_command_node.
MAIN_MISSION_STATUS_TOPIC = "/main_mission/status"
MAIN_MISSION_SIM_STATUS_TOPIC = "/main_mission_sim/status"

# radio_command_node's own status (JSON std_msgs/String, 1 Hz): serial
# port, last packet, last command and its ACK -- for local debugging on the
# Jetson (`ros2 topic echo`); the GCS gets the same facts over the radio.
RADIO_STATUS_TOPIC = "/radio/status"
