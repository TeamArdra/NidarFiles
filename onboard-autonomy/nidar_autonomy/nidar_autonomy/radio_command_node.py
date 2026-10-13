"""Receives operator START/ABORT over the MicroLR900 radio plugged into
the Jetson (USB serial) and hands them to the normal command path:

    serial -> MavlinkStreamParser -> CommandGate -> /gcs/mission_select
           -> /gcs/command -> command_node (validation) -> hover mission

The radio terminates HERE, at the Jetson -- not at the Pixhawk. This node
never talks to MAVROS services and never arms or moves anything itself;
it only publishes exactly "start"/"abort" on /gcs/command, the same topic
and validation path every operator command already uses (Hard Safety
Rules 1 and 5). Every command and its ACK/NACK is logged with a timestamp
(Hard Safety Rule 4).

Also sends a 1 Hz radio HEARTBEAT (custom_mode = mission state code) so
the GCS can show RADIO LINK UP/DOWN, and relays telemetry to the GCS over
the radio -- FCU state, battery, local position, attitude, FCU status
text and the hover mission's progress (radio_telemetry.py). There is no
Wi-Fi link; this radio is the GCS's only view of the drone. /radio/status
is published for local debugging on the Jetson.

Parameters:
  serial_port          primary device; default is the radio's stable by-id path
  fallback_serial_port used if the primary doesn't exist (default /dev/ttyUSB0)
  baud                 115200 (must match the MicroLR900's USB baud)
  dry_run              true = log and ACK, but publish nothing (radio bench test)
  telemetry_rate_hz    position/attitude rate over the radio (default 2.0);
                       0 disables telemetry (commands and heartbeat only)
  lidar_rate_hz        LiDAR scans (/scan) relayed over the radio per second as
                       72-sector OBSTACLE_DISTANCE (default 1.0; 0 = off)
  map_rate_hz          SLAM map packets (TUNNEL, <= ~145 B) per second while
                       Cartographer publishes /map (default 2.0; 0 = off)
  map_cell_m           map cell size sent to the GCS (default 0.25 m)
  arena_rate_hz        1 m arena grid packets (TUNNEL ARENA_GRID, arena_grid.py)
                       per second (default 1.0; 0 = off). The grid itself is
                       recomputed from the latest /map every arena_period_s
                       (default 3 s) on a worker thread.
  allow_param_write    true = the GCS Setup page may WRITE Pixhawk parameters
                       (bench only, `start_jetson.sh --setup`; refused while
                       armed or a mission runs). Reads are always answered.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, StatusText
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import String

from .param_bridge_logic import addressed_to_jetson, refusal, typed_value
from .radio_command_logic import CommandGate, Decision, Readiness
from .arena_grid import Alignment, OccupancyMap, build_grid, estimate_alignment
from .map_grid import coarsen
from .radio_telemetry import (
    DEFAULT_ARENA_RATE_HZ,
    DEFAULT_LIDAR_RATE_HZ,
    DEFAULT_MAP_RATE_HZ,
    DEFAULT_RATE_HZ,
    TelemetryRelay,
)
from .rplidar_protocol import sectors_from_laserscan
from .telem_command_codec import (
    JETSON_COMPONENT_ID,
    JETSON_SYSTEM_ID,
    MISSION_CODES,
    MISSION_STATE_CODES,
    MISSION_STATE_UNKNOWN,
    MAV_SEVERITY_WARNING,
    MSG_ID_COMMAND_LONG,
    MSG_ID_PARAM_REQUEST_READ,
    MSG_ID_PARAM_SET,
    PARAM_TEXT_PREFIX,
    REASON_NAMES,
    MavlinkStreamParser,
    decode_command_long,
    decode_param_request,
    encode_command_ack,
    encode_heartbeat,
    encode_param_value,
    encode_statustext,
    parse_radio_command,
    statustext_chunks,
)
from .topics import (
    BATTERY_TOPIC,
    COMMAND_TOPIC,
    FCU_STATE_TOPIC,
    FCU_STATUSTEXT_TOPIC,
    HOVER_STATUS_TOPIC,
    IMU_TOPIC,
    LOCAL_POSITION_TOPIC,
    LOCAL_VELOCITY_TOPIC,
    MAP_TOPIC,
    MISSION_SELECT_TOPIC,
    MOTOR_TEST_STATUS_TOPIC,
    RADIO_STATUS_TOPIC,
    SCAN_TOPIC,
)

# TODO(hardware): confirm on the Jetson with `ls -l /dev/serial/by-id/`.
# If another CP2102 device (e.g. a LiDAR adapter) produces the same name,
# switch to the /dev/serial/by-path/ name of the radio's USB port.
DEFAULT_SERIAL_PORT = (
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)
DEFAULT_FALLBACK_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200

# Each mission node's status topic: its liveness signal and its state.
MISSION_STATUS_TOPICS = {
    "hover": HOVER_STATUS_TOPIC,
    "motor_test": MOTOR_TEST_STATUS_TOPIC,
}

# MAVROS mirrors every FCU parameter as a ROS parameter of this node.
MAVROS_PARAM_NODE = "/mavros/param"

_FCU_STATE_STALE_S = 3.0
_MISSION_STATUS_STALE_S = 3.0
_REOPEN_INTERVAL_S = 2.0


def _now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S")


class RadioCommandNode(Node):
    def __init__(self) -> None:
        super().__init__("radio_command_node")
        self._port = self.declare_parameter("serial_port", DEFAULT_SERIAL_PORT).value
        self._fallback_port = self.declare_parameter(
            "fallback_serial_port", DEFAULT_FALLBACK_PORT
        ).value
        self._baud = int(self.declare_parameter("baud", DEFAULT_BAUD).value)
        self._dry_run = bool(self.declare_parameter("dry_run", False).value)
        self._telemetry_rate_hz = float(
            self.declare_parameter("telemetry_rate_hz", DEFAULT_RATE_HZ).value
        )
        self._allow_param_write = bool(self.declare_parameter("allow_param_write", False).value)
        self._lidar_rate_hz = float(self.declare_parameter("lidar_rate_hz", DEFAULT_LIDAR_RATE_HZ).value)
        self._map_rate_hz = float(self.declare_parameter("map_rate_hz", DEFAULT_MAP_RATE_HZ).value)
        self._map_cell_m = float(self.declare_parameter("map_cell_m", 0.25).value)
        self._map_processed_at: Optional[float] = None
        self._arena_rate_hz = float(self.declare_parameter("arena_rate_hz", DEFAULT_ARENA_RATE_HZ).value)
        self._arena_period_s = float(self.declare_parameter("arena_period_s", 3.0).value)
        self._latest_map: Optional[OccupancyGrid] = None
        self._arena_alignment: Optional[Alignment] = None
        self._param_text_id = 0

        self._gate = CommandGate()
        self._parser = MavlinkStreamParser()
        self._serial = None
        self._serial_lock = threading.Lock()
        self._open_port: Optional[str] = None
        self._tx_seq = 0
        self._stop = threading.Event()

        self._fcu_connected = False
        self._fcu_armed: Optional[bool] = None
        self._fcu_state_at: Optional[float] = None
        # mission id -> (state, monotonic time of its last status)
        self._missions: dict = {}
        # The mission the GCS sees in the heartbeat / telemetry: the one
        # last STARTed (hover until then).
        self._active_mission = "hover"
        self._last_rx_at: Optional[float] = None
        self._last_command: Optional[dict] = None
        self._telemetry = TelemetryRelay(
            self._next_seq, time.monotonic(), self._lidar_rate_hz, self._map_rate_hz, self._arena_rate_hz
        )
        # SLAM pose of the LiDAR = TF map -> laser (Cartographer), if running.
        self._tf_buffer = None
        try:
            from tf2_ros import Buffer, TransformListener

            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        except Exception as exc:  # noqa: BLE001 -- tf2_ros missing: map without pose
            self.get_logger().warning(f"[{_now()}] no TF ({exc!r}) -- SLAM pose not sent")

        self._command_pub = self.create_publisher(String, COMMAND_TOPIC, 10)
        self._select_pub = self.create_publisher(String, MISSION_SELECT_TOPIC, 10)
        self._status_pub = self.create_publisher(String, RADIO_STATUS_TOPIC, 10)
        self._param_get = self.create_client(GetParameters, f"{MAVROS_PARAM_NODE}/get_parameters")
        self._param_set = self.create_client(SetParameters, f"{MAVROS_PARAM_NODE}/set_parameters")

        best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(State, FCU_STATE_TOPIC, self._on_fcu_state, best_effort)
        for mission_id, topic in MISSION_STATUS_TOPICS.items():
            self.create_subscription(
                String, topic, lambda msg, m=mission_id: self._on_mission_status(m, msg), 10
            )
        self.create_subscription(BatteryState, BATTERY_TOPIC, self._on_battery, best_effort)
        self.create_subscription(PoseStamped, LOCAL_POSITION_TOPIC, self._on_pose, best_effort)
        self.create_subscription(TwistStamped, LOCAL_VELOCITY_TOPIC, self._on_velocity, best_effort)
        self.create_subscription(Imu, IMU_TOPIC, self._on_imu, best_effort)
        self.create_subscription(StatusText, FCU_STATUSTEXT_TOPIC, self._on_statustext, best_effort)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, best_effort)
        # BEST_EFFORT + VOLATILE subscribes to any /map publisher QoS.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=1
        )
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, map_qos)

        self.create_timer(1.0, self._send_heartbeat_and_status)
        if self._telemetry_rate_hz > 0:
            self.create_timer(1.0 / self._telemetry_rate_hz, self._send_telemetry)
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        # The arena grid takes ~1 s of CPU on the Jetson: its own thread, so
        # telemetry timers don't stall (commands are on the reader thread).
        self._arena_worker = None
        if self._arena_rate_hz > 0 and self._map_rate_hz > 0:
            self._arena_worker = threading.Thread(target=self._arena_loop, daemon=True)
            self._arena_worker.start()

        self.get_logger().info(
            f"[{_now()}] radio_command_node: port {self._port} "
            f"(fallback {self._fallback_port}) @ {self._baud}, Jetson id "
            f"{JETSON_SYSTEM_ID}/{JETSON_COMPONENT_ID}"
            + f", telemetry {self._telemetry_rate_hz} Hz"
            + (" -- DRY RUN: commands are ACKed but NOT published" if self._dry_run else "")
            + (" -- SETUP: Pixhawk parameter WRITES allowed (disarmed only)" if self._allow_param_write else "")
        )

    # -- ROS inputs -------------------------------------------------------------

    def _on_fcu_state(self, msg: State) -> None:
        self._fcu_connected = bool(msg.connected)
        self._fcu_armed = bool(msg.armed)
        self._fcu_state_at = time.monotonic()
        self._telemetry.update_fcu_state(
            self._fcu_state_at, msg.connected, msg.armed, msg.guided, msg.mode, msg.system_status
        )

    def _on_mission_status(self, mission_id: str, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(status, dict) and status.get("execution_mode") == "real":
            now = time.monotonic()
            self._missions[mission_id] = (status.get("state"), now)
            if mission_id == self._active_mission:
                self._telemetry.update_mission_status(now, status)

    def _on_battery(self, msg: BatteryState) -> None:
        self._telemetry.update_battery(time.monotonic(), msg.voltage, msg.current, msg.percentage)

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._telemetry.update_position(time.monotonic(), p.x, p.y, p.z)

    def _on_velocity(self, msg: TwistStamped) -> None:
        v = msg.twist.linear
        self._telemetry.update_velocity(time.monotonic(), v.x, v.y, v.z)

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        self._telemetry.update_orientation(time.monotonic(), q.w, q.x, q.y, q.z)

    def _on_scan(self, msg: LaserScan) -> None:
        sectors = sectors_from_laserscan(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max
        )
        self._telemetry.update_scan(
            time.monotonic(), sectors, int(msg.range_min * 100), int(msg.range_max * 100)
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg  # the arena worker picks up the newest one
        now = time.monotonic()
        if self._map_rate_hz <= 0 or (self._map_processed_at is not None and now - self._map_processed_at < 1.0):
            return  # at most one reduction per second
        self._map_processed_at = now
        info = msg.info
        meta, cells = coarsen(
            msg.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y, self._map_cell_m,
        )
        self._telemetry.update_map(now, meta, cells)

    def _arena_loop(self) -> None:
        done = None
        while not self._stop.wait(self._arena_period_s):
            msg = self._latest_map
            if msg is None or msg is done:
                continue
            done = msg
            try:
                info = msg.info
                m = OccupancyMap(
                    msg.data, info.width, info.height, info.resolution,
                    info.origin.position.x, info.origin.position.y,
                )
                al = estimate_alignment(m, near=self._arena_alignment)
                if al is None:
                    continue
                if self._arena_alignment is None:
                    self.get_logger().info(
                        f"[{_now()}] arena grid aligned: {math.degrees(al.theta_rad):.1f} deg, "
                        f"grid lines at {al.offset_u:.2f} / {al.offset_v:.2f} m"
                    )
                self._arena_alignment = al
                grid = build_grid(m, al)
                if grid is not None:
                    self._telemetry.update_arena(time.monotonic(), grid)
            except Exception as exc:  # noqa: BLE001 -- never take the radio node down
                self.get_logger().error(f"[{_now()}] arena grid failed: {exc!r}")

    def _update_slam_pose(self) -> None:
        if self._tf_buffer is None:
            return
        try:
            t = self._tf_buffer.lookup_transform("map", "laser", Time())
        except Exception:  # noqa: BLE001 -- no SLAM running / not yet
            return
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._telemetry.update_slam_pose(
            time.monotonic(), t.transform.translation.x, t.transform.translation.y, yaw
        )

    def _on_statustext(self, msg: StatusText) -> None:
        self._telemetry.add_fcu_statustext(msg.severity, msg.text)

    def _readiness(self) -> Readiness:
        now = time.monotonic()
        fcu_fresh = self._fcu_state_at is not None and now - self._fcu_state_at < _FCU_STATE_STALE_S
        missions = {
            mission_id: state
            for mission_id, (state, at) in self._missions.items()
            if now - at < _MISSION_STATUS_STALE_S
        }
        return Readiness(fcu_connected=fcu_fresh and self._fcu_connected, missions=missions)

    # -- serial -----------------------------------------------------------------

    def _open(self) -> bool:
        import serial  # python3-serial; imported here so a missing package is a clear runtime error

        for port in (self._port, self._fallback_port):
            if not port or not os.path.exists(port):
                continue
            try:
                handle = serial.Serial(port, self._baud, timeout=0.1)
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"[{_now()}] cannot open {port}: {exc}")
                continue
            with self._serial_lock:
                self._serial = handle
                self._open_port = port
            self.get_logger().info(f"[{_now()}] radio serial open: {port} @ {self._baud}")
            return True
        return False

    def _close(self, why: str) -> None:
        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:  # noqa: BLE001 -- closing a dead port
                    pass
            self._serial = None
            self._open_port = None
        self.get_logger().error(f"[{_now()}] radio serial closed: {why}")

    def _read_loop(self) -> None:
        warned = False
        while not self._stop.is_set():
            if self._serial is None:
                if not self._open():
                    if not warned:
                        self.get_logger().error(
                            f"[{_now()}] radio not found at {self._port} or "
                            f"{self._fallback_port}; retrying every {_REOPEN_INTERVAL_S}s"
                        )
                        warned = True
                    self._stop.wait(_REOPEN_INTERVAL_S)
                    continue
                warned = False
            try:
                data = self._serial.read(256)
            except Exception as exc:  # noqa: BLE001 -- unplugged / I/O error
                if self._stop.is_set():
                    break  # shutting down: the port was closed under us
                self._close(repr(exc))
                continue
            if not data:
                continue
            for frame in self._parser.feed(data):
                self._last_rx_at = time.monotonic()
                if frame.msgid == MSG_ID_COMMAND_LONG:
                    self._on_command_long(frame)
                elif frame.msgid in (MSG_ID_PARAM_REQUEST_READ, MSG_ID_PARAM_SET):
                    self._on_param_request(frame)

    def _write(self, frame: bytes) -> None:
        with self._serial_lock:
            if self._serial is None:
                return
            try:
                self._serial.write(frame)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"[{_now()}] radio write failed: {exc!r}")

    def _next_seq(self) -> int:
        self._tx_seq = (self._tx_seq + 1) & 0xFF
        return self._tx_seq

    # -- commands ---------------------------------------------------------------

    def _on_command_long(self, frame) -> None:
        cmd = parse_radio_command(
            decode_command_long(frame.payload), JETSON_SYSTEM_ID, JETSON_COMPONENT_ID
        )
        if cmd is None:
            return
        sender = (frame.sysid, frame.compid)
        decision = self._gate.decide(cmd, sender, self._readiness())
        reason = REASON_NAMES.get(decision.reason, str(decision.reason))
        tag = (
            f"{cmd.command.upper()} mission_code={cmd.mission_code} nonce={cmd.nonce} "
            f"from {sender[0]}/{sender[1]}"
        )

        if decision.duplicate:
            self.get_logger().info(f"[{_now()}] radio resend of {tag}; re-ACK result={decision.result} {reason}")
        elif decision.forward is None:
            self.get_logger().warning(f"[{_now()}] radio {tag} REJECTED: {reason}")
        else:
            self.get_logger().warning(f"[{_now()}] radio {tag} ACCEPTED -> {COMMAND_TOPIC}")
            self._forward(decision, cmd.nonce)

        self._write(
            encode_command_ack(
                decision.result, decision.reason, cmd.nonce, sender[0], sender[1], self._next_seq()
            )
        )
        if not decision.duplicate:
            self._last_command = {
                "command": cmd.command,
                "mission_code": cmd.mission_code,
                "nonce": cmd.nonce,
                "result": decision.result,
                "reason": reason,
                "time": _now(),
            }

    # -- bench parameter access (GCS Setup page) -------------------------------

    def _on_param_request(self, frame) -> None:
        req = decode_param_request(frame)
        if req is None or not addressed_to_jetson(req):
            return
        readiness = self._readiness()
        reason = refusal(
            req,
            self._allow_param_write,
            readiness.fcu_connected,
            self._fcu_armed if readiness.fcu_connected else None,
            readiness.missions,
        )
        what = f"param {req.kind} {req.name}" + ("" if req.value is None else f" = {req.value:g}")
        if reason:
            self.get_logger().warning(f"[{_now()}] radio {what} REFUSED: {reason}")
            self._param_reply_error(req.name, reason)
            return
        self.get_logger().warning(f"[{_now()}] radio {what}")
        self._param_fetch(req.name, lambda value, is_int: self._param_after_get(req, value, is_int))

    def _param_after_get(self, req, value: Optional[float], is_int: bool) -> None:
        if value is None:
            return  # error already reported
        if req.kind == "read":
            self._write(encode_param_value(req.name, value, is_int, self._next_seq()))
            return
        new_value, error = typed_value(req.value, is_int)
        if error:
            self._param_reply_error(req.name, error)
            return
        param_value = ParameterValue()
        if is_int:
            param_value.type = ParameterType.PARAMETER_INTEGER
            param_value.integer_value = int(new_value)
        else:
            param_value.type = ParameterType.PARAMETER_DOUBLE
            param_value.double_value = float(new_value)
        request = SetParameters.Request()
        request.parameters = [Parameter(name=req.name, value=param_value)]
        self.get_logger().warning(
            f"[{_now()}] PARAM WRITE {req.name}: {value:g} -> {new_value:g}"
        )
        future = self._param_set.call_async(request)
        future.add_done_callback(lambda f: self._param_after_set(req.name, f))

    def _param_after_set(self, name: str, future) -> None:
        try:
            result = future.result().results[0]
        except Exception as exc:  # noqa: BLE001 -- service failure
            self._param_reply_error(name, f"set failed: {exc!r}")
            return
        if not result.successful:
            self._param_reply_error(name, f"FCU refused: {result.reason or 'no reason given'}")
            return
        # Read back what the FCU now holds -- that is what the GCS shows.
        self._param_fetch(name, lambda value, is_int: self._param_confirm(name, value, is_int))

    def _param_confirm(self, name: str, value: Optional[float], is_int: bool) -> None:
        if value is None:
            return
        self.get_logger().warning(f"[{_now()}] PARAM {name} is now {value:g} on the FCU")
        self._write(encode_param_value(name, value, is_int, self._next_seq()))

    def _param_fetch(self, name: str, then) -> None:
        """Current FCU value of `name` via MAVROS: then(value, is_integer),
        or reports the error and calls then(None, False)."""
        if not self._param_get.service_is_ready():
            self._param_reply_error(name, "MAVROS parameter service not available")
            then(None, False)
            return
        request = GetParameters.Request()
        request.names = [name]

        def done(future) -> None:
            try:
                pv = future.result().values[0]
            except Exception as exc:  # noqa: BLE001
                self._param_reply_error(name, f"read failed: {exc!r}")
                then(None, False)
                return
            if pv.type == ParameterType.PARAMETER_INTEGER:
                then(float(pv.integer_value), True)
            elif pv.type == ParameterType.PARAMETER_DOUBLE:
                then(float(pv.double_value), False)
            else:
                self._param_reply_error(name, "unknown parameter (or MAVROS has not loaded the list yet)")
                then(None, False)

        self._param_get.call_async(request).add_done_callback(done)

    def _param_reply_error(self, name: str, reason: str) -> None:
        text = f"{PARAM_TEXT_PREFIX}{name}: {reason}"
        self.get_logger().warning(f"[{_now()}] {text}")
        chunks = statustext_chunks(text)
        text_id = 0
        if len(chunks) > 1:
            # 0x8000+ so it never matches the telemetry relay's chunk ids
            self._param_text_id = 0x8000 + (self._param_text_id + 1) % 0x7FFF
            text_id = self._param_text_id
        for chunk_seq, chunk in enumerate(chunks):
            self._write(
                encode_statustext(MAV_SEVERITY_WARNING, chunk, self._next_seq(), text_id, chunk_seq)
            )

    def _forward(self, decision: Decision, nonce: int) -> None:
        if self._dry_run:
            self.get_logger().warning(f"[{_now()}] DRY RUN: not publishing {decision.forward!r}")
            return
        if decision.forward == "start":
            self._active_mission = decision.mission_id
            selection = {"mission_id": decision.mission_id, "source": "radio", "nonce": nonce}
            self._select_pub.publish(String(data=json.dumps(selection)))
        self._command_pub.publish(String(data=decision.forward))

    # -- periodic ---------------------------------------------------------------

    def _send_heartbeat_and_status(self) -> None:
        readiness = self._readiness()
        state = readiness.missions.get(self._active_mission)
        code = MISSION_STATE_CODES.get(state or "", MISSION_STATE_UNKNOWN)
        self._write(
            encode_heartbeat(code, self._next_seq(), MISSION_CODES.get(self._active_mission, 0))
        )
        now = time.monotonic()
        status = {
            "serial_open": self._serial is not None,
            "serial_port": self._open_port,
            "last_rx_age_s": None if self._last_rx_at is None else round(now - self._last_rx_at, 1),
            "fcu_connected": readiness.fcu_connected,
            "missions": dict(readiness.missions),
            "active_mission": self._active_mission,
            "last_command": self._last_command,
            "dry_run": self._dry_run,
            "allow_param_write": self._allow_param_write,
        }
        self._status_pub.publish(String(data=json.dumps(status)))

    def _send_telemetry(self) -> None:
        if self._serial is None:
            return
        self._update_slam_pose()
        for frame in self._telemetry.tick(time.monotonic()):
            self._write(frame)

    def destroy_node(self) -> bool:
        # Stop the reader thread BEFORE closing its port: closing a port a
        # thread is blocked reading made pyserial raise a TypeError on every
        # shutdown (reported on the Jetson). Same order as lidar_node.
        self._stop.set()
        self._reader.join(timeout=1.0)
        if self._arena_worker is not None:
            self._arena_worker.join(timeout=2.0)
        self._close("node shutdown")
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = RadioCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
