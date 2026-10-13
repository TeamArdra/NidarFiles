"""Constants of the MicroLR900 command-radio protocol, GCS side.

Mirrors onboard-autonomy/nidar_autonomy/nidar_autonomy/telem_command_codec.py
(the Jetson side, which documents the full wire format). The two files
must be kept in sync -- the repos are separate, so this is a copy, not an
import.

    GCS 255/190 --COMMAND_LONG(MAV_CMD_USER_1)--> Jetson 1/191
        param1 START=1 / ABORT=2, param2 nonce, param3 magic 4242,
        param4 mission code, param7 protocol version
    Jetson --COMMAND_ACK--> GCS
        result MAV_RESULT, progress = reason code, result_param2 = nonce
    Jetson --HEARTBEAT (1 Hz)--> GCS
        custom_mode = (mission code << 8) | mission state code

Telemetry, Jetson -> GCS (there is no Wi-Fi link; see app/radio_telemetry.py):
    from 1/191  HEARTBEAT (mission state), STATUSTEXT (hover mission detail),
                NAMED_VALUE_FLOAT (hover progress, HOVER_VALUE_KEYS)
    from 1/1    the FCU's state as MAVROS reports it, relayed by the Jetson:
                HEARTBEAT (armed/guided/ArduCopter mode, only while MAVROS
                is connected), SYS_STATUS (battery), LOCAL_POSITION_NED,
                ATTITUDE_QUATERNION (q1..q4 = w x y z), STATUSTEXT
    Position/velocity/attitude are in the ROS ENU/FLU frame MAVROS uses,
    not NED.
"""

MAV_CMD_USER_1 = 31010
MAV_RESULT_ACCEPTED = 0

JETSON_SYSTEM_ID = 1
JETSON_COMPONENT_ID = 191
FCU_RELAY_COMPONENT_ID = 1
GCS_SYSTEM_ID = 255
GCS_COMPONENT_ID = 190

NIDAR_START = 1.0
NIDAR_ABORT = 2.0
NIDAR_MAGIC = 4242.0
PROTOCOL_VERSION = 2

MAX_NONCE = 2**24 - 1  # float32 params represent integers exactly up to here

REASON_NAMES = {
    0: "OK",
    1: "UNKNOWN_MISSION",
    2: "BAD_PROTOCOL_VERSION",
    3: "MISSION_NOT_READY",
    4: "FCU_NOT_CONNECTED",
    5: "MISSION_BUSY",
}

MISSION_STATE_NAMES = {
    0: "idle",
    1: "preflight",
    2: "setting_guided",
    3: "arming",
    4: "taking_off",
    5: "hovering",
    6: "landing",
    7: "complete",
    8: "aborted",
    9: "failed",
    10: "pilot_override",
    11: "testing",
    255: "unknown",
}

# Heartbeat high byte -> mission id (app/missions.py radio codes). 0 = none known.
MISSION_NAMES_BY_CODE = {1: "hover", 2: "motor_test"}

MAV_MODE_FLAG_GUIDED_ENABLED = 8
MAV_MODE_FLAG_SAFETY_ARMED = 128

ARDUCOPTER_MODE_NAMES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
    6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
    15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB",
    20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG",
    25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL", 28: "TURTLE",
}

# NAMED_VALUE_FLOAT name -> hover mission status field (FlightTestStatusResponse)
HOVER_VALUE_KEYS = {
    "hv_alt": "current_altitude_m",
    "hv_tgt": "target_altitude_m",
    "hv_dur": "duration_s",
    "hv_elap": "elapsed_hover_s",
}

STATUSTEXT_CHUNK_LEN = 50

# Bench parameter access (Setup page; Jetson needs start_jetson.sh --setup
# for writes): GCS -> Jetson PARAM_REQUEST_READ / PARAM_SET by name;
# Jetson -> GCS PARAM_VALUE (value now on the FCU) or a STATUSTEXT
# "PARAM: <NAME>: <reason>" when refused.
PARAM_TEXT_PREFIX = "PARAM: "

# SLAM map over the radio (Jetson map_grid.py): MAVLink TUNNEL payload types.
# MAP_ROWS: <H cell_cm><h origin_x_cm><h origin_y_cm><B width><B height>
#           <B row0><B nrows> + cells packed 2 bits each (0 unknown, 1 free,
#           2 occupied; 4 per byte, first cell in the low bits), row-major,
#           row 0 = lowest y.
# SLAM_POSE: <h x_cm><h y_cm><h yaw_cdeg> in the map frame (yaw CCW from +x).
MAP_ROWS_TYPE = 0x8001
SLAM_POSE_TYPE = 0x8002
# ARENA_GRID (Jetson arena_grid.py): the map as the organiser's 1 m grid --
# every cell edge WALL/OPEN/UNKNOWN, split into parts of <= 128 bytes:
#   <B gen><B part><B nparts><h theta_cdeg><H offset_u_mm><H offset_v_mm>
#   <h entry_u><h entry_v><b col0><b row0><B cols><B rows>
#   + edges packed 2 bits each (0 unknown, 1 open, 2 wall; 4 per byte, first
#   in the low bits). Edge stream: west edges row by row (rows x (cols + 1)),
#   then south edges row by row ((rows + 1) x cols); part k carries edges
#   k * ARENA_EDGES_PER_PART onwards. gen changes whenever the grid does.
# Arena frame: u = x cos(theta) + y sin(theta), v = -x sin(theta) + y cos(theta)
# (map frame x, y); grid lines at u = offset_u + k, v = offset_v + k; cell
# (col, row) spans u in offset_u + entry_u + col .. +1 (same for v / row).
# Cell (0, 0) is where the drone took off.
ARENA_GRID_TYPE = 0x8003
ARENA_EDGES_PER_PART = (128 - 17) * 4
